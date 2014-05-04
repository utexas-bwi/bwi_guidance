#!/usr/bin/env python

import bwi_tools
import math
import random
import roslib
import rospy
import threading
import time
import yaml

from tf.transformations import quaternion_from_euler

from bwi_guidance_msgs.msg import ExperimentStatus, RobotInfoArray
from bwi_guidance_msgs.srv import UpdateExperiment, UpdateExperimentResponse, \
                         UpdateExperimentServer, UpdateExperimentServerRequest
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, \
                            GetModelState, GetModelStateRequest
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

def get_pose_msg_from_2d(x, y, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0
    pose.orientation = Quaternion(*quaternion_from_euler(0,0,yaw))
    return pose

def distance(a, b):
    return math.sqrt(((a[1]-b[1]) * (a[1]-b[1])) + ((a[0]-b[0]) * (a[0]-b[0])))

def orientation(a, b):
    return math.atan2(a[1] - b[1], a[0] - b[0])

def normalize_angle(y):
    return math.atan2(math.sin(y), math.cos(y))

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
                msg.robot_positioning_enabled = \
                        self.controller.robot_positioning_enabled
                msg.uid = self.controller.experiment_uid
                msg.total_reward = self.controller.reward
                msg.current_display_text = self.controller.experiment_text
                msg.instance_number = self.controller.instance_number
                if (msg.instance_number >= 0 and 
                    msg.instance_number < self.controller.num_instances):
                    msg.instance_name = \
                            self.controller.instance_names[msg.instance_number]
                else:
                    msg.instance_name = ''
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
        self.result_file = rospy.get_param("~result_file")
        self.data_directory = rospy.get_param("~data_dir",
                roslib.packages.get_pkg_dir("bwi_guidance") + "/data")

        self.experiment_uid = rospy.get_param("~uid") 
        self.experiment_uname = rospy.get_param("~name") 
        self.experiment_uemail = rospy.get_param("~email") 
        self.use_heuristic = rospy.get_param("~use_heuristic")

        # See if an experiment server is running
        rospy.loginfo("Waiting for service: server/update_server")
        try:
            rospy.wait_for_service("server/update_server", 5.0)
            self.update_server = rospy.ServiceProxy(
                    'server/update_server', UpdateExperimentServer)
            self.server_available = True
            rospy.loginfo("Server AVAILABLE!")
        except rospy.exceptions.ROSException:
            self.server_available = False
            rospy.loginfo("Server NOT AVAILABLE!")

        # Get Gazebo services
        rospy.loginfo("Waiting for service: gazebo/set_model_state")
        rospy.wait_for_service("gazebo/set_model_state")
        self.set_gazebo_model_state = \
                rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
        rospy.loginfo("Service acquired: gazebo/set_model_state")

        rospy.loginfo("Waiting for service: gazebo/get_model_state")
        rospy.wait_for_service("gazebo/get_model_state")
        self.get_gazebo_model_state = \
                rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
        rospy.loginfo("Service acquired: gazebo/get_model_state")

        rospy.loginfo("Waiting for service: gazebo/pause_physics")
        rospy.wait_for_service("gazebo/pause_physics")
        self.pause_gazebo = rospy.ServiceProxy("gazebo/pause_physics", Empty)
        rospy.loginfo("Service acquired: gazebo/pause_physics")

        rospy.loginfo("Waiting for service: gazebo/unpause_physics")
        rospy.wait_for_service("gazebo/unpause_physics")
        self.unpause_gazebo = \
                rospy.ServiceProxy("gazebo/unpause_physics", Empty)
        rospy.loginfo("Service acquired: gazebo/unpause_physics")

        # Setup the experiment interface
        self.interface = ExperimentInterface(self)

        # Get some constants across all instances
        self.person_id = self.experiment['person_id']
        self.ball_id = self.experiment['ball_id']

        # Initialize some variables used for the interface
        self.reward = 0
        self.experiment_text = ''
        self.experiment_success = False
        self.experiment_ready = False
        self.robot_positioner_ready = False
        self.odometry_ready = False
        self.experiment_ready_timer_started = False

        # Initialize some default values for a single experiment
        self.instance_number = 0
        self.num_instances = 0
        self.instance_in_progress = False
        self.robot_positioning_enabled = False
        self.instance_success = False
        self.start_next_instance = False
        self.paused = False
        self.instance_distance_covered = 0
        self.prev_loc = [0, 0]
        self.wait_for_first_odom = False

        # Setup the odometry subscriber
        self.odometry_subscriber = rospy.Subscriber(self.person_id + '/odom', 
                Odometry, self.odometry_callback)

        # Setup the robot positioner subscriber
        self.robot_positioner_subscriber = rospy.Subscriber('robot_positions',
                RobotInfoArray, self.robot_positioner_callback)

        # A couple of locks
        self.modify_experiment_lock = threading.Lock()
        self.modify_instance_lock = threading.Lock()

        # Start the experiment
        self.start_experiment()

    def start_instance(self, log_file_name, experiment_data, instance_number):

        self.modify_instance_lock.acquire()

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

        # setup odometry thread to write out odometry values
        self.log_file_name = log_file_name
        self.odometry_log = []
        self.robot_log = []

        # Save experiment start time
        # self.experiment_start_time = rospy.get_time()
        self.experiment_max_duration = experiment_data['max_duration']

        # Setup interactions with user
        self.experiment_tutorial = experiment_data['is_tutorial']
        if self.experiment_tutorial:
            self.max_tutorial_time = experiment_data['tutorial_duration']
        self.experiment_text = "Loading..."

        # self.instance_in_progress = True
        self.robot_positioning_enabled = True
        self.instance_success = False

        self.wait_for_first_odom = True

        self.modify_instance_lock.release()

    def stop_instance(self, success):

        self.modify_instance_lock.acquire()
        self.instance_in_progress = False
        self.robot_positioning_enabled = False

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
            self.instance_results.append(time_diff)
            self.instance_results.append(self.instance_distance_covered)
        else:
            if (success):
                if self.instances[self.instance_number + 1]['is_tutorial']:
                    self.experiment_text = "Awesome! You found the goal! "\
                            + "There are more TUTORIALS to go!"
                else:
                    self.experiment_text = "Awesome! You've finished the "\
                            + "tutorials!";
                reward = 50
            else:
                current_time = rospy.get_time()
                time_diff = current_time - self.experiment_start_time
                self.experiment_text = "Oh no! You were unable to find the "\
                        + "ball quickly enough (" + str(time_diff) + " "\
                        + "seconds). You need to find the ball in "\
                        + str(self.max_tutorial_time) + " seconds. "\
                        + "Let's try the tutorial again!"
                reward = 0

        self.experiment_text += \
                " Hit the ENTER key or press Continue!"
        self.reward += reward

        # Write log file
        log_data = {'odometry': self.odometry_log,
                    'robots': self.robot_log}
        with open(self.log_file_name, 'w') as log_file:
            log_file.write(yaml.dump(log_data, default_flow_style=False))
            log_file.close()

        self.modify_instance_lock.release()

    def ready_the_experiment(self):
        rospy.loginfo("Odometry and Robot Positioner are both ready. Starting the experiment!")
        self.experiment_ready = True

    def odometry_callback(self, odom):

        self.modify_instance_lock.acquire()

        # Once first robot position message is received, check if odometry is also ready and start the experiment
        self.odometry_ready = True
        if not self.experiment_ready_timer_started and self.robot_positioner_ready:
            self.experiment_ready_timer_started = True
            self.experiment_ready_timer = \
                threading.Timer(5.0, self.ready_the_experiment)
            self.experiment_ready_timer.start()

        if (not self.instance_in_progress):
            # Instance not ready 
            self.modify_instance_lock.release()
            return

        ros_time = odom.header.stamp.to_sec()
        loc = [odom.pose.pose.position.x, odom.pose.pose.position.y]

        if self.wait_for_first_odom:
            self.instance_distance_covered = 0
            self.wait_for_first_odom = False
        else:
            self.instance_distance_covered = self.instance_distance_covered +\
                    math.hypot(loc[1] - self.prev_loc[1], loc[0] -
                               self.prev_loc[0]) 
        self.prev_loc = loc

        # record position
        self.odometry_log.append([loc[0],loc[1],ros_time]);

        # check if the person is close to the ball
        distance_from_goal = distance(loc, self.goal_location)
        if distance_from_goal < 1.0:
            self.instance_success = True

        self.modify_instance_lock.release()

    def robot_positioner_callback(self, msg):

        self.modify_instance_lock.acquire()

        # Once first robot position message is received, check if odometry is also ready and start the experiment
        self.robot_positioner_ready = True
        if not self.experiment_ready_timer_started and self.odometry_ready:
            self.experiment_ready_timer_started = True
            self.experiment_ready_timer = \
                threading.Timer(5.0, self.ready_the_experiment)
            self.experiment_ready_timer.start()

        if self.robot_positioning_enabled and not self.instance_in_progress:
            if msg.ready:
                # Robot positioning is now ready. Start the experiment
                self.instance_in_progress = True
                self.experiment_start_time = rospy.get_time()
                if self.experiment_tutorial:
                    self.experiment_text = "#" + str(self.instance_number + 1) + " "\
                            + "The TUTORIAL has started! Use this tutorial to become "\
                            + "comfortable with moving the person around."
                else:
                    self.experiment_text = "#" + str(self.instance_number + 1) + " "\
                            + "The EXPERIMENT has started! Find the red ball as "\
                            + "quickly as possible!"
                self.ping_server()
            else:
                self.modify_instance_lock.release()
                return

        if not self.instance_in_progress:
            self.modify_instance_lock.release()
            return

        ros_time = msg.header.stamp.to_sec()
        robots = [{'loc': [r.pose.position.x, r.pose.position.y],
                   'direction': r.direction,
                   'is_ok': r.is_ok} for r in msg.robots]
        self.robot_log.append({'time': ros_time, 'robots': robots})
            
        self.modify_instance_lock.release()

    def start(self):

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
        self.instance_results = []
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

        # Write users file for visualization
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

        # Write results file for graph computation
        result_file = open(self.result_file, "a")
        if self.use_heuristic:
            result_file.write("heuristic,")
        else:
            result_file.write("vi,")
        count = 0
        for instance_time in self.instance_results:
            result_file.write(str(instance_time))
            if count != len(self.instance_results) - 1:
                result_file.write(",")
            count = count + 1
        result_file.write('\n')
        result_file.close()

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
        if (not self.instance_in_progress and 
            self.instance_number != len(self.instances)):
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
