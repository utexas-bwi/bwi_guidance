#!/usr/bin/env python

import bwi_tools
import cv
import cv2
import math
import numpy
import random
import roslib
import rospy
import threading
import time
import yaml

from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler

from bwi_msgs.msg import ExperimentStatus
from bwi_msgs.srv import PositionRobot, PositionRobotRequest, \
                         UpdateExperiment, UpdateExperimentResponse, \
                         UpdateExperimentServer, UpdateExperimentServerRequest
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, \
                            GetModelState, GetModelStateRequest
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_srvs.srv import Empty

def get_pose_msg_from_2d(x, y, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0
    pose.orientation = Quaternion(*quaternion_from_euler(0,0,yaw))
    return pose

def readImage(location):
    return cv2.imread(location)

def distance(a, b):
    return math.sqrt(((a[1]-b[1]) * (a[1]-b[1])) + ((a[0]-b[0]) * (a[0]-b[0])))

def orientation(a, b):
    return math.atan2(a[1] - b[1], a[0] - b[0])

def normalize_angle(y):
    return math.atan2(math.sin(y), math.cos(y))

def produce_directed_arrow(up_arrow, yaw):
    height, width, channels = up_arrow.shape
    center = (width / 2, height / 2)
    rotation_matrix = cv2.getRotationMatrix2D(center, yaw * 180 / math.pi, 1.0)
    rotated_image = cv2.warpAffine(up_arrow, rotation_matrix, (width, height))

    height_ratio = 119.0 / height
    width_ratio = 159.0 / width
    min_ratio = min(height_ratio, width_ratio)

    resized_image = cv2.resize(rotated_image, (0,0), fx=min_ratio, fy=min_ratio)
    image = numpy.empty((120,160,3), numpy.uint8)
    top = (image.shape[0] - resized_image.shape[0]) / 2
    bottom = image.shape[0] - resized_image.shape[0] - top
    left = (image.shape[1] - resized_image.shape[1]) / 2
    right = image.shape[1] - resized_image.shape[1] - left

    image = cv2.copyMakeBorder(resized_image, top, bottom, left, right, 
                               cv2.BORDER_CONSTANT, image, (0,0,0))
    return image

def compute_average_angle(angle1, angle2):
    vec1 = [math.cos(angle1), math.sin(angle1)]
    vec2 = [math.cos(angle2), math.sin(angle2)]
    average_vec = [vec1[0] + vec2[0], vec1[1] + vec2[1]]
    return math.atan2(average_vec[1], average_vec[0])

def check_close_poses(pose1, pose2):
    if (math.fabs(pose1.position.x - pose2.position.x) < 0.1 and
        math.fabs(pose1.position.y - pose2.position.y) < 0.1):
        return True
    return False

class ExperimentInterface(threading.Thread):

    def __init__(self, controller):
        threading.Thread.__init__(self)
        self.experiment_status_publisher = rospy.Publisher(
                '~experiment_status', ExperimentStatus)
        self.update_experiment = rospy.Service(
                '~update_experiment', UpdateExperiment, 
                self.handle_experiment_update)
        self.interface_lock = threading.Lock()
        self.controller = controller

    def handle_experiment_update(self, req):
        resp = UpdateExperimentResponse()
        if req.pause_experiment:
            resp.result = self.controller.pause_experiment()
            if not resp.result:
                resp.status = "Cannot pause the experiment right now!"
        elif req.unpause_experiment:
            resp.result = self.controller.unpause_experiment()
            if not resp.result:
                resp.status = "Cannot unpause the experiment right now!"
        elif req.continue_experiment:
            resp.result = self.controller.continue_to_next_instance()
            if not resp.result:
                resp.status = "Cannot start next experiment right now!"
        return resp

    def run(self):
        rate = bwi_tools.WallRate(5) #5Hz
        try:
            while not rospy.is_shutdown():
                self.interface_lock.acquire()
                msg = ExperimentStatus()
                msg.instance_in_progress = self.controller.instance_in_progress
                msg.uid = self.controller.experiment_uid
                msg.total_reward = self.controller.reward
                msg.current_display_text = self.controller.experiment_text
                msg.instance_number = self.controller.instance_number
                msg.total_experiments = self.controller.num_instances
                msg.pause_enabled = msg.instance_in_progress
                msg.paused = self.controller.paused
                msg.continue_enabled = ((not msg.instance_in_progress) and
                        (msg.instance_number != msg.total_experiments))
                msg.experiment_success = self.controller.experiment_success
                msg.experiment_ready = self.controller.experiment_ready
                self.experiment_status_publisher.publish(msg)
                self.interface_lock.release()
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
        rospy.loginfo("Controller Interface thread shutting down...")

class RobotScreenPublisher(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.robot_image_publisher = {}
        self.robot_image = {}
        self.robot_image_lock = {}
        self.bridge = CvBridge()

    def add_robot(self, robotid):
        self.robot_image_publisher[robotid] = rospy.Publisher(
                robotid + '/image', Image)
        self.robot_image[robotid] = numpy.zeros((120,160,3), numpy.uint8)
        self.robot_image_lock[robotid] = threading.Lock()

    def update(self, robotid, new_image):
        self.robot_image_lock[robotid].acquire()
        self.robot_image[robotid] = new_image
        self.robot_image_lock[robotid].release()

    def run(self):
        rate = bwi_tools.WallRate(5) # 5hz
        try:
            while not rospy.is_shutdown():
                for robotid in self.robot_image_publisher.keys():
                    self.robot_image_lock[robotid].acquire()
                    image = self.robot_image[robotid]
                    cv_image = cv.fromarray(image)
                    image_msg = self.bridge.cv_to_imgmsg(cv_image, "bgr8")
                    self.robot_image_lock[robotid].release()
                    self.robot_image_publisher[robotid].publish(image_msg)
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
        rospy.loginfo("Robot screen publisher thread shutting down...")

class ExperimentController:

    def __init__(self):

        rospy.init_node('experiment')

        # Get ROS Parameters
        self.graph_file = rospy.get_param("~graph_file")
        try:
            self.graph = yaml.load(open(self.graph_file,'r'))
        except Exception as e:
            rospy.logerr("Unable to open graph file: %s"%e.what())

        self.experiment_file = rospy.get_param("~experiment_file")
        try:
            self.experiment = yaml.load(open(self.experiment_file,'r'))
        except Exception as e:
            rospy.logerr("Unable to open experiment file: %s"%e.what())

        self.user_file = rospy.get_param("~user_file")
        self.data_directory = rospy.get_param("~data_dir",
                roslib.packages.get_pkg_dir("bwi_exp1") + "/data")

        self.experiment_uid = rospy.get_param("~uid") 
        self.experiment_uname = rospy.get_param("~name") 
        self.experiment_uemail = rospy.get_param("~email") 

        # Get ROS Services
        rospy.loginfo("Waiting for service: position")
        rospy.wait_for_service("position")
        self.position_robot = rospy.ServiceProxy('position', PositionRobot)
        rospy.loginfo("Service acquired: position")

        # See if an experiment server is running
        rospy.loginfo("Waiting for service: /server/update_server")
        try:
            rospy.wait_for_service("/server/update_server", 5.0)
            self.update_server = rospy.ServiceProxy(
                    '/server/update_server', UpdateExperimentServer)
            self.server_available = True
            rospy.loginfo("Server AVAILABLE!")
        except rospy.exceptions.ROSException:
            self.server_available = False
            rospy.loginfo("Server NOT AVAILABLE!")

        # Get Gazebo services
        rospy.loginfo("Waiting for service: /gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_gazebo_model_state = \
                rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        rospy.loginfo("Service acquired: /gazebo/set_model_state")

        rospy.loginfo("Waiting for service: /gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/get_model_state")
        self.get_gazebo_model_state = \
                rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        rospy.loginfo("Service acquired: /gazebo/get_model_state")

        rospy.loginfo("Waiting for service: /gazebo/pause_physics")
        rospy.wait_for_service("/gazebo/pause_physics")
        self.pause_gazebo = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        rospy.loginfo("Service acquired: /gazebo/pause_physics")

        rospy.loginfo("Waiting for service: /gazebo/unpause_physics")
        rospy.wait_for_service("/gazebo/unpause_physics")
        self.unpause_gazebo = \
                rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        rospy.loginfo("Service acquired: /gazebo/unpause_physics")

        # Setup components required for publishing directed arrows to the
        # robot screens
        images_dir = roslib.packages.get_pkg_dir("bwi_exp1") + "/images"
        self.arrow = readImage(images_dir + "/Up.png")
        self.image_none = numpy.zeros((120,160,3), numpy.uint8)

        self.robot_image_publisher = RobotScreenPublisher()
        self.robots = self.experiment['robots']
        for i, robot in enumerate(self.robots):
            robot_id = robot['id']
            self.robot_image_publisher.add_robot(robot_id)

        # Track robots currently being used in an experiment instance 
        self.robots_in_instance = []
        self.robot_locations = []
        self.robot_images = []

        # Setup the experiment interface
        self.interface = ExperimentInterface(self)

        # Get some constants across all instances
        self.person_id = self.experiment['person_id']
        self.robots = self.experiment['robots']
        self.ball_id = self.experiment['ball_id']

        # Initialize some variables used for the interface
        self.reward = 0
        self.experiment_text = ''
        self.experiment_success = False
        self.experiment_ready = False
        self.experiment_ready_timer_started = False

        # Initialize some default values for a single experiment
        self.instance_number = 0
        self.num_instances = 0
        self.instance_in_progress = False
        self.instance_success = False
        self.start_next_instance = False
        self.paused = False

        # Setup the odometry subscriber
        self.odometry_subscriber = rospy.Subscriber(self.person_id + '/odom', 
                Odometry, self.odometry_callback)

        # A couple of locks
        self.modify_experiment_lock = threading.Lock()
        self.modify_instance_lock = threading.Lock()

        # Start the experiment
        self.start_experiment()

    def start_instance(self, log_file_name, experiment_data, instance_number):

        self.modify_instance_lock.acquire()

        # Unpause gazebo before teleportation
        self.unpause_gazebo()

        # Teleport person to correct place, however pause and unpause
        start_x = experiment_data['start_x']
        start_y = experiment_data['start_y']
        start_yaw = experiment_data['start_yaw']
        start_pose = get_pose_msg_from_2d(start_x, start_y, yaw=start_yaw)
        result = self.teleport_entity(self.person_id, start_pose)

        # Teleport the goal ball to its correct position
        ball_x = experiment_data['ball_x']
        ball_y = experiment_data['ball_y']
        ball_pose = get_pose_msg_from_2d(ball_x, ball_y, 0)
        result = self.teleport_entity(self.ball_id, ball_pose)
        self.goal_location = [ball_x, ball_y]

        # Teleport all the robots to their respective positions
        self.path = experiment_data['path']
        self.robots_in_instance = \
                [{'id': p['id'], 'path_position': i} 
                 for i, p in enumerate(self.path) if p['robot']]
        self.extra_robots = experiment_data['extra_robots']
        self.robot_locations = [None] * len(self.robots)
        self.robot_images = [None] * len(self.robots)
        for i, robot in enumerate(self.robots):
            if i < len(self.robots_in_instance):
                # This robit is being used in this instance
                # Find a position for it
                path_position = self.robots_in_instance[i]['path_position']

                # Get position and orientation of robot from C++ based service
                req = PositionRobotRequest()
                req.from_id = -1
                req.to_id = -1
                req.at_id = self.robots_in_instance[i]['id']
                if path_position != 0:
                    req.from_id = self.path[path_position - 1]['id']
                    arriving_from = self.get_location_at_graph_id(req.from_id)
                else:
                    req.from_pt.x = start_x
                    req.from_pt.y = start_y
                    req.from_pt.z = 0
                    arriving_from = [start_x, start_y]
                if path_position != len(self.path) - 1:
                    req.to_id = self.path[path_position + 1]['id']
                    going_to = self.get_location_at_graph_id(req.to_id)
                else:
                    req.to_pt.x = ball_x
                    req.to_pt.y = ball_y
                    req.to_pt.z = 0
                    going_to = [ball_x, ball_y]

                resp = self.position_robot(req)
                robot_loc = [resp.loc.x, resp.loc.y]
                robot_yaw = resp.yaw

                # Get orientation of the arrow on the screen
                # If the point in the graph is too close, select the next point
                destination_distance = distance(going_to, robot_loc)
                path_position = path_position + 1
                while destination_distance < 3:
                    if path_position != len(self.path) - 1:
                        going_to = self.get_location_at_graph_id(
                            self.path[path_position + 1]['id'])
                    else:
                        going_to = [ball_x, ball_y]
                    destination_distance = distance(going_to, robot_loc)
                    path_position = path_position + 1

                destination_yaw = orientation(going_to, robot_loc)
                change_in_yaw = destination_yaw - robot_yaw
                change_in_yaw = normalize_angle(change_in_yaw) 

                # Compute the arrow based on direction
                robot_image = produce_directed_arrow(self.arrow, change_in_yaw)

            elif (i - len(self.robots_in_instance)) < len(self.extra_robots): 
                # This is an extra (distraction) robot. Move it into place
                extra_robot = self.extra_robots[i-len(self.robots_in_instance)]
                
                robot_loc = [extra_robot['loc_x'], extra_robot['loc_y']]
                robot_yaw = extra_robot['yaw']
                robot_image = self.image_none

            else: # Move a robot not used in the experiment
                robot_loc = self.robots[i]['default_loc']
                robot_yaw = 0
                robot_image = self.image_none

            # Perform the actual teleport and image update
            robot_pose = get_pose_msg_from_2d(*robot_loc, yaw=robot_yaw)
            result = self.teleport_entity(self.robots[i]['id'], robot_pose)
            self.robot_images[i] = robot_image
            self.robot_locations[i] = robot_loc

        # setup odometry thread to write out odometry values
        self.log_file = open(log_file_name, 'w')
        self.log_file.write("odometry: \n")

        # Save experiment start time
        self.experiment_start_time = rospy.get_time()
        self.experiment_max_duration = experiment_data['max_duration']

        # Setup interactions with user
        self.experiment_tutorial = experiment_data['is_tutorial']
        if self.experiment_tutorial:
            self.max_tutorial_time = experiment_data['tutorial_duration']
            self.experiment_text = "#" + str(instance_number + 1) + " "\
                    + "The TUTORIAL has started! Use this tutorial to become "\
                    + "comfortable with moving the person around."
        else:
            self.experiment_text = "#" + str(instance_number + 1) + " "\
                    + "The EXPERIMENT has started! Find the red ball as "\
                    + "quickly as possible!"

        self.instance_in_progress = True
        self.instance_success = False

        self.modify_instance_lock.release()

    def stop_instance(self, success):

        self.modify_instance_lock.acquire()
        self.instance_in_progress = False

        if not self.experiment_tutorial:
            current_time = rospy.get_time()
            time_diff = current_time - self.experiment_start_time
            reward_time = self.experiment_max_duration - time_diff

            # Send message to user
            if (success):
                reward = int(reward_time) + 50
                self.experiment_text = "You found the goal in " \
                        + str(time_diff) + " seconds and earned " \
                        + str(reward) + " points!"
            else:
                reward = 25
                self.experiment_text = "Oh no! It looks like you ran out of "\
                        + "time with this experiment. We've awarded you "\
                        + str(reward) + " points."
        else:
            if (success):
                if self.instances[self.instance_number + 1]['is_tutorial']:
                    self.experiment_text = "Awesome! You found the goal! "\
                            + "There are more TUTORIALS to go!"
                else:
                    self.experiment_text = "Awesome! You've finished the "\
                            + "tutorials! Hit the enter key or press the "\
                            + "continue button to proceed to the EXPERIMENTS!"
                reward = 50
            else:
                current_time = rospy.get_time()
                time_diff = current_time - self.experiment_start_time
                self.experiment_text = "Oh no! You were unable to find the "\
                        + "ball quickly enough (" + str(time_diff) + " "\
                        + "seconds). You need to find the ball in "\
                        + str(self.experiment_tutorial_duration) + " seconds. "\
                        + "Let's try the tutorial again!"
                reward = 0

        self.experiment_text += \
                " Hit the ENTER key or press Continue to try again!"
        self.reward += reward

        # setup odometry thread to stop writing out odometry values
        self.log_file.write("success: " + str(success))
        self.log_file.close()

        for robot in self.robots:
            robot_pose = get_pose_msg_from_2d(*robot['default_loc'], yaw=0)
            self.teleport_entity(robot['id'], robot_pose) 

        # Leave gazebo paused to prevent the user from moving the character
        # around
        self.pause_gazebo()

        self.modify_instance_lock.release()

    def ready_the_experiment(self):
        self.experiment_ready = True

    def odometry_callback(self, odom):

        self.modify_instance_lock.acquire()

        # Once first odometry message is received, ready the experiment in 10s
        if not self.experiment_ready_timer_started:
            self.experiment_ready_timer_started = True
            self.experiment_ready_timer = \
                threading.Timer(10.0, self.ready_the_experiment)
            self.experiment_ready_timer.start()

        if (not self.instance_in_progress):
            # This instance has completed. We just received an extra odom 
            # message from Gazebo before it got completely paused.
            self.modify_instance_lock.release()
            return

        ros_time = odom.header.stamp.to_sec()
        loc = [odom.pose.pose.position.x, odom.pose.pose.position.y]

        # record position
        self.log_file.write("  - [" + str(loc[0]) + ", " + str(loc[1]) + ", "
                            + str(ros_time) + "]\n")

        # check if the person is near a robot
        for i, robot in enumerate(self.robots):
            if i < len(self.robots_in_instance):
                # This is non-distraction robot that is a part of the instance
                robot_loc = self.robot_locations[i]
                distance_from_robot = distance(loc, robot_loc)
                if distance_from_robot < 3.0:
                    self.robot_image_publisher.update(robot['id'],
                                                      self.robot_images[i])
                else:
                    self.robot_image_publisher.update(robot['id'], 
                                                      self.image_none)

        # check if the person is close to the ball
        distance_from_goal = distance(loc, self.goal_location)
        if distance_from_goal < 1.0:
            self.instance_success = True

        self.modify_instance_lock.release()

    def start(self):

        self.robot_image_publisher.start()
        self.interface.start()

        # Use wall rate here as you don't want the loop to stop while gazebo is
        # paused
        rate = bwi_tools.WallRate(10) # 10hz

        while not rospy.is_shutdown():

            # Get ROS time here in case Gazebo was paused
            if self.instance_in_progress:
                current_time = rospy.get_time()
                time_taken = current_time - self.experiment_start_time

            self.modify_experiment_lock.acquire()

            if self.start_next_instance:
                # Should start next instance
                log_file = self.log_file_prefix + \
                        self.instance_names[self.instance_number]
                self.start_instance(log_file,
                    self.instances[self.instance_number], self.instance_number)
                self.start_next_instance = False

            elif self.instance_in_progress and self.instance_success:
                if self.experiment_tutorial:
                    if time_taken > self.max_tutorial_time:
                        # Cannot complete a tutorial this slow, redo
                        self.stop_instance(False)
                    else:
                        self.stop_instance(True)
                        self.instance_number = self.instance_number + 1
                else: 
                    self.stop_instance(True)
                    self.instance_number = self.instance_number + 1
                if self.instance_number == len(self.instances):
                    # All done! Nothing to see here
                    self.finalize_experiment()
            elif (self.instance_in_progress and 
                  time_taken > self.experiment_max_duration):
                # The instance timed out. If tutorial then redo.
                self.stop_instance(False)
                if not self.experiment_tutorial:
                    self.instance_number = self.instance_number + 1
                if self.instance_number == len(self.instances):
                    # All done! Nothing to see here
                    self.finalize_experiment()

            self.modify_experiment_lock.release()
            rate.sleep()

    def get_location_at_graph_id(self, id):
        for entry in self.graph:
            if entry['id'] == id:
                return [entry['x'], entry['y']]
        raise IndexError, "GraphId %d does not exist in %s"%(id,self.graph_file)

    def teleport_entity(self, entity, pose):
        """
        Teleports an model named by entity to give pose. Sometimes gazebo 
        teleportation fails, so a number of repeated attempts are made. An
        error is printed if teleportation fails after repeated attempts.

        THIS FUNCTION CANNOT BE CALLED WHILE GAZEBO IS PAUSED!! Any attempts
        to check location while gazebo is paused will fail. Use carefully

        """
        count = 0
        attempts = 5
        location_verified = False
        while count < attempts and not location_verified:
            get_state = GetModelStateRequest()
            get_state.model_name = entity
            resp = self.get_gazebo_model_state(get_state)
            if check_close_poses(resp.pose, pose):
                location_verified = True
            else:
                set_state = SetModelStateRequest()
                set_state.model_state.model_name = entity
                set_state.model_state.pose = pose
                resp = self.set_gazebo_model_state(set_state)
                if not resp.success:
                    rospy.logwarn("Failed attempt at moving object")
            count = count + 1
        if not location_verified:
            rospy.logerr("Unable to move " + entity + " to " + str(pose) 
                     + " after " + attempts + " attempts.")
        return location_verified

    def start_experiment(self):
        """
        Compute the order in which the instance groups need to be performed
        If a group has order != -1, then it is placed at location <order>
        in the instance group order. If the order is -1, then all -1 instance 
        groups are shuffled and placed at the end

        EXAMPLE - The following instance groups with the specified order
        prefix: tutorial, order: 0
        prefix: first, order: 1
        prefix: dont_care_1, order: -1
        prefix: dont_care_2, order: -1
        can result in 1 of 2 orderings:
        [tutorial, first, dont_care_1, dont_care_2] OR
        [tutorial, first, dont_care_2, dont_care_1]

        """
        self.instance_group_order = \
                [None] * len(self.experiment['instance_groups'])
        randomly_ordered_groups = []
        highest_ordered_group = -1
        for instance_group in self.experiment['instance_groups']:
            group_name = instance_group['prefix']
            group_order = instance_group['order']
            if group_order != -1: 
                self.instance_group_order[group_order] = group_name
                highest_ordered_group = max(highest_ordered_group, group_order)
            else:
                randomly_ordered_groups.append(group_name)
        random_groups_start = highest_ordered_group + 1
        random.shuffle(randomly_ordered_groups)
        self.instance_group_order[random_groups_start:] = \
                randomly_ordered_groups
        rospy.loginfo("Selected the following group order: %s"%
                      str(self.instance_group_order))

        # Load instances based on the instance group order
        self.instance_names = []
        self.instances = []
        self.instance_group_count = []
        for instance_group_name in self.instance_group_order:
            for instance_group in self.experiment['instance_groups']:
                if (instance_group['prefix'] != instance_group_name):
                    continue
                num_instances = len(instance_group['instances'])
                self.instance_names.extend(
                    [instance_group_name + "_" + str(i) 
                     for i in range(num_instances)]
                )
                self.instances.extend(instance_group['instances'])
                self.instance_group_count.append(num_instances)

        # calculate the total number of instances from all groups
        rospy.loginfo("Instances will be run in following order: %s"%
                      str(self.instance_names))
        self.num_instances = len(self.instances)
        rospy.loginfo("There are a total of %i instances"%
                      self.num_instances)
        self.log_file_prefix = \
                self.data_directory + "/" + self.experiment_uid + "_"
        rospy.loginfo("Using log prefix: %s"%self.log_file_prefix)  

        # Prepare to start the first instance
        self.instance_number = 0
        self.reward = 0
        rospy.loginfo("Starting experiment for user: " + self.experiment_uid)

    def finalize_experiment(self):
        user_file = open(self.user_file, "a")
        user_file.write("- user: " + self.experiment_uid + "\n")
        user_file.write("  name: " + self.experiment_uname + "\n")
        user_file.write("  email: " + self.experiment_uemail + "\n")
        user_file.write("  experiment_order:\n")
        for i in range(len(self.instance_group_order)):
            user_file.write("    - name: " + 
                            self.instance_group_order[i] + "\n")
            user_file.write("      num: " + 
                            str(self.instance_group_count[i]) + "\n")
        user_file.close()
        rospy.loginfo("Finalized instances for user: " + self.experiment_uid)
        self.experiment_success = True
        self.experiment_text = "Huzzah. You are all done. Thanks for "\
                + " participating in this experiment!"
        time.sleep(10.0)
        self.unlock_server()

    def ping_server(self):
        if self.server_available:
            req = UpdateExperimentServerRequest()
            req.keep_alive = True
            req.uid = self.experiment_uid
            self.update_server(req)

    def unlock_server(self):
        if self.server_available:
            req = UpdateExperimentServerRequest()
            req.unlock_experiment = True
            req.uid = self.experiment_uid
            self.update_server(req)

    def continue_to_next_instance(self):
        success = False
        self.modify_experiment_lock.acquire()
        if not self.instance_in_progress:
            self.start_next_instance = True
            self.ping_server()
            success = True
        self.modify_experiment_lock.release()
        return success

    def pause_experiment(self):
        success = False
        if self.instance_in_progress and not self.paused:
            self.pause_gazebo()
            self.paused = True
            success = True
        return success

    def unpause_experiment(self):
        success = False
        if self.instance_in_progress and self.paused:
            self.unpause_gazebo()
            self.paused = False
            self.ping_server()
            success = True
        return success

if __name__ == '__main__':
    try:
        controller = ExperimentController()
        controller.start()
    except rospy.ROSInterruptException:
        pass
