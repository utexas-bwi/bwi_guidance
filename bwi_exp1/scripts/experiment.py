#!/usr/bin/env python

import roslib; roslib.load_manifest('bwi_exp1')
import rospy

import yaml
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from bwi_msgs.msg import ExperimentStatus
from bwi_msgs.srv import PositionRobot, PositionRobotRequest, UpdatePluginState, UpdateExperiment, UpdateExperimentResponse
from gazebo_msgs.srv import SetModelState, SetModelStateRequest

import cv, cv2, numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import threading
import math
import random,string
import copy

def getPoseMsgFrom2dData(x, y, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0
    pose.orientation = Quaternion(*quaternion_from_euler(0,0,yaw))
    return pose

def readImage(location):
    return cv2.imread(location)

def produceDirectedArrow(up_arrow, yaw):
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

    image = cv2.copyMakeBorder(resized_image, top, bottom, left, right, cv2.BORDER_CONSTANT, image, (0,0,0))
    return image

def id_generator(size=6, chars=string.ascii_uppercase + string.digits):
    return ''.join(random.choice(chars) for x in range(size))

def compute_average_angle(angle1, angle2):
    vec1 = [math.cos(angle1), math.sin(angle1)]
    vec2 = [math.cos(angle2), math.sin(angle2)]
    average_vec = [vec1[0] + vec2[0], vec1[1] + vec2[1]]
    return math.atan2(average_vec[1], average_vec[0])

class ExperimentInterface:

    def __init__(self, experiment_controller):
        self.experiment_text = ""
        self.reward = 0.0
        self.reward_in_previous_experiment = 0.0
        self.previous_experiment_reset = False
        self.previous_experiment_successfully_finished = False
        self.experiment_interface_lock = threading.Lock()
        self.experiment_status_publisher = rospy.Publisher('~status', ExperimentStatus)
        self.update_experiment_server = rospy.Service('~update_experiment', UpdateExperiment, self.handleUpdateExperimentRequest)
        
        self.experiment_controller = experiment_controller

    def handleUpdateExperimentRequest(self, req):
        resp = UpdateExperimentResponse()
        if req.lock_experiment:
            resp.result, resp.uid = self.experiment_controller.startExperimentsForNewUser(req.name, req.email)
            if not resp.result:
                resp.status = "The experiment server is already locked. It cannot be locked right now!"
            else:
                self.experiment_interface_lock.acquire()
                self.reward = 0
                self.previous_experiment_reset = False
                self.previous_experiment_successfully_finished = False
                self.experiment_interface_lock.release()
        elif req.pause_experiment:
            resp.result = self.experiment_controller.pauseCurrentExperiment()
            if not resp.result:
                resp.status = "Cannot pause the experiment right now!"
            else:
                self.publishStatus()
        elif req.unpause_experiment:
            resp.result = self.experiment_controller.unpauseCurrentExperiment()
            if not resp.result:
                resp.status = "Cannot unpause the experiment right now!"
        elif req.continue_experiment:
            resp.result = self.experiment_controller.startNextExperiment()
            if not resp.result:
                resp.status = "Cannot start next experiment right now!"
        return resp

    def startExperiment(self, text):
        self.experiment_interface_lock.acquire()
        self.experiment_text = text
        self.experiment_interface_lock.release()

    def stopCurrentExperiment(self, reward, text):
        self.experiment_interface_lock.acquire()
        self.experiment_text = text
        self.reward_in_previous_experiment = reward
        self.reward += self.reward_in_previous_experiment
        self.experiment_interface_lock.release()

    def resetExperiment(self):
        self.experiment_interface_lock.acquire()
        self.reward = -1
        self.previous_experiment_reset = True
        self.experiment_interface_lock.release()

    def stopExperiment(self):
        self.experiment_interface_lock.acquire()
        self.previous_experiment_successfully_finished = True
        self.experiment_interface_lock.release()

    def publishStatus(self):
        self.experiment_interface_lock.acquire()
        msg = ExperimentStatus()
        msg.locked = self.experiment_controller.experiment_server_locked
        msg.experiment_in_progress = self.experiment_controller.experiment_in_progress
        msg.uid = self.experiment_controller.experiment_uid
        msg.total_reward = self.reward
        msg.current_display_text = self.experiment_text
        msg.experiment_number = self.experiment_controller.experiment_number
        msg.total_experiments = self.experiment_controller.num_experiments
        msg.reward_in_previous_experiment = self.reward_in_previous_experiment
        msg.pause_enabled = msg.locked and msg.experiment_in_progress
        msg.continue_enabled = msg.locked and (not msg.experiment_in_progress)
        msg.reset_warning_text = ""
        if self.experiment_controller.reset_timer:
            if msg.pause_enabled:
                msg.reset_warning_text = "Please UNPAUSE within the next " + str(self.experiment_controller.reset_time_remaining) + " seconds to avoid timing out the entire experiment!"
            elif msg.continue_enabled:
                msg.reset_warning_text = "Please CONTINUE within the next " + str(self.experiment_controller.reset_time_remaining) + " seconds to avoid timing out the entire experiment!"
        msg.previous_experiment_reset = self.previous_experiment_reset
        msg.previous_experiment_successfully_finished = self.previous_experiment_successfully_finished
        self.experiment_status_publisher.publish(msg)
        self.experiment_interface_lock.release()

class RobotScreenPublisher(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.robot_image_publisher = {}
        self.robot_image = {}
        self.robot_image_lock = {}
        self.bridge = CvBridge()

    def addRobot(self, robotid):
        self.robot_image_publisher[robotid] = rospy.Publisher(robotid + '/image', Image)
        self.robot_image[robotid] = numpy.zeros((120,160,3), numpy.uint8)
        self.robot_image_lock[robotid] = threading.Lock()

    def updateImage(self, robotid, new_image):
        self.robot_image_lock[robotid].acquire()
        self.robot_image[robotid] = new_image
        self.robot_image_lock[robotid].release()

    def run(self):
        r = rospy.Rate(5) # 5hz
        try:
            while not rospy.is_shutdown():
                for robotid in self.robot_image_publisher.keys():
                    self.robot_image_lock[robotid].acquire()
                    image = self.robot_image[robotid]
                    cv_image = cv.fromarray(image)
                    image_msg = self.bridge.cv_to_imgmsg(cv_image, "bgr8")
                    self.robot_image_lock[robotid].release()
                    self.robot_image_publisher[robotid].publish(image_msg)
                r.sleep()
        except rospy.ROSInterruptException:
            pass

class ExperimentController:

    def __init__(self):
        rospy.init_node('experiment_controller')

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

        self.person_id = self.experiment['person_id']
        self.robots = self.experiment['robots']
        self.ball_id = self.experiment['ball_id']

        self.control_experiments_file = rospy.get_param("~control_experiments_file")
        self.test_experiments_file = rospy.get_param("~test_experiments_file")
        self.tutorials_file = rospy.get_param("~tutorials_file")
        try:
            self.tutorials = yaml.load(open(self.tutorials_file, 'r'))
            for exp in self.tutorials:
                exp['is_tutorial'] = True
            self.control_experiments = yaml.load(open(self.control_experiments_file,'r'))
            for exp in self.control_experiments:
                exp['is_tutorial'] = False
            self.test_experiments = yaml.load(open(self.test_experiments_file,'r'))
            for exp in self.test_experiments:
                exp['is_tutorial'] = False
        except Exception as e:
            rospy.logerr("Unable to open experiment file: %s"%str(e))

        self.data_directory = rospy.get_param("~data_dir")

        # Get the robot positioner service
        rospy.loginfo("Waiting for service: position")
        rospy.wait_for_service("position")
        self.position_robot = rospy.ServiceProxy('position', PositionRobot)
        rospy.loginfo("Service acquired: position")

        # Get the teleporter server
        rospy.loginfo("Waiting for service: /gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.teleport = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        rospy.loginfo("Service acquired: /gazebo/set_model_state")

        rospy.loginfo("Waiting for service: /gazebo/pause_physics")
        rospy.wait_for_service("/gazebo/pause_physics")
        self.pauseGazebo = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        rospy.loginfo("Service acquired: /gazebo/pause_physics")

        rospy.loginfo("Waiting for service: /gazebo/unpause_physics")
        rospy.wait_for_service("/gazebo/unpause_physics")
        self.unpauseGazebo = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        rospy.loginfo("Service acquired: /gazebo/unpause_physics")
        self.unpauseGazebo()

        # Get the person pause service
        rospy.loginfo("Waiting for service: " + self.person_id + "/update_state")
        rospy.wait_for_service(self.person_id + "/update_state")
        self.pause_person_plugin = rospy.ServiceProxy(self.person_id + "/update_state", UpdatePluginState)
        rospy.loginfo("Service acquired: " + self.person_id + "/update_state")
        self.pause_person_plugin(True)

        # Read the arrows
        images_dir = roslib.packages.get_pkg_dir("bwi_web") + "/images"
        self.arrow = readImage(images_dir + "/Up.png")
        self.image_none = numpy.zeros((120,160,3), numpy.uint8)

        # Setup the Robot image publisher
        self.robot_image_publisher = RobotScreenPublisher()
        for i, robot in enumerate(self.robots):
            robot_id = robot['id']
            self.robot_image_publisher.addRobot(robot_id)

        # Setup some robot defaults
        self.robots_in_experiment = []
        self.robot_locations = []
        self.robot_images = []

        # Setup the experiment text publisher
        self.experiment_interface = ExperimentInterface(self)

        # Setup some default values for modifying experiments 
        self.experiment_lock_mutex = threading.Lock()
        self.experiment_server_locked = False

        # Setup some default values for a single experiment
        self.experiment_number = 0
        self.num_experiments = 0
        self.experiment_uid = ""
        self.experiment_in_progress = False
        self.experiment_success = False
        self.start_next_experiment = False
        self.allow_logging = True

        # Setup the odometry subscriber
        sub = rospy.Subscriber(self.person_id + '/odom', Odometry, self.odometryCallback)

        # Log file lock
        self.log_lock = threading.Lock()

        # Reset timer stuff
        self.reset_timer = None
        self.reset_time_remaining = 0
        self.reset_timer_lock = threading.Lock()

    def startExperiment(self, log_file_name, experiment_data, experiment_number):

        # Pause gazebo during teleportation
        self.pauseGazebo() # time also stops

        # Teleport person to correct place, however pause and unpause
        start_x = experiment_data['start_x']
        start_y = experiment_data['start_y']
        start_yaw = experiment_data['start_yaw']
        start_pose = getPoseMsgFrom2dData(start_x, start_y, yaw=start_yaw)
        result = self.teleportEntity(self.person_id, start_pose)

        # Teleport the goal ball to its correct position
        ball_x = experiment_data['ball_x']
        ball_y = experiment_data['ball_y']
        ball_pose = getPoseMsgFrom2dData(ball_x, ball_y, 0)
        result = self.teleportEntity(self.ball_id, ball_pose)
        self.goal_location = [ball_x, ball_y]

        # Teleport all the robots to their respective positions
        self.path = experiment_data['path']
        self.robots_in_experiment = [{'id': p['id'], 'path_position': i} for i, p in enumerate(self.path) if p['robot']]
        self.extra_robots = experiment_data['extra_robots']
        self.robot_locations = [None] * len(self.robots)
        self.robot_images = [None] * len(self.robots)
        for i, robot in enumerate(self.robots):
            if i < len(self.robots_in_experiment): #one of the robors being used in this experiment
                path_position = self.robots_in_experiment[i]['path_position']

                # Get position and orientation of robot from C++ based service
                req = PositionRobotRequest()
                req.from_id = -1
                req.to_id = -1
                req.at_id = self.robots_in_experiment[i]['id']
                if path_position != 0:
                    req.from_id = self.path[path_position - 1]['id']
                    arriving_from = self.getGraphLocation(req.from_id)
                else:
                    req.from_pt.x = start_x
                    req.from_pt.y = start_y
                    req.from_pt.z = 0
                    arriving_from = [start_x, start_y]
                if path_position != len(self.path) - 1:
                    req.to_id = self.path[path_position + 1]['id']
                    going_to = self.getGraphLocation(req.to_id)
                else:
                    req.to_pt.x = ball_x
                    req.to_pt.y = ball_y
                    req.to_pt.z = 0
                    going_to = [ball_x, ball_y]

                resp = self.position_robot(req)
                robot_loc = [resp.loc.x, resp.loc.y]
                robot_yaw = resp.yaw

                # Get orientation of the arrow on the screen
                # If the next point in the graph is too close, select the next
                # point
                destination_distance = math.sqrt((going_to[1] - robot_loc[1])*(going_to[1] - robot_loc[1]) + (going_to[0] - robot_loc[0])*(going_to[0] - robot_loc[0]))
                path_position = path_position + 1
                while destination_distance < 3:
                    if path_position != len(self.path) - 1:
                        going_to = self.getGraphLocation(self.path[path_position + 1]['id'])
                    else:
                        going_to = [ball_x, ball_y]
                    destination_distance = math.sqrt((going_to[1] - robot_loc[1])*(going_to[1] - robot_loc[1]) + (going_to[0] - robot_loc[0])*(going_to[0] - robot_loc[0]))
                    path_position = path_position + 1

                destination_yaw = math.atan2(going_to[1] - robot_loc[1], going_to[0] - robot_loc[0])
                change_in_yaw = destination_yaw - robot_yaw
                change_in_yaw = math.atan2(math.sin(change_in_yaw), math.cos(change_in_yaw)) #normalize angle

                # Compute the arrow based on direction
                robot_image = produceDirectedArrow(self.arrow, change_in_yaw)

            elif i - len(self.robots_in_experiment) < len(self.extra_robots): # Move a distraction robot into place
                robot_loc = [self.extra_robots[i - len(self.robots_in_experiment)]['loc_x'], self.extra_robots[i - len(self.robots_in_experiment)]['loc_y']]
                robot_yaw = self.extra_robots[i - len(self.robots_in_experiment)]['yaw']
                robot_image = self.image_none

            else: # Move a robot not used in the experiment
                robot_loc = self.robots[i]['default_loc']
                robot_yaw = 0
                robot_image = self.image_none

            # Perform the actual teleport and image update
            robot_pose = getPoseMsgFrom2dData(*robot_loc, yaw=robot_yaw)
            result = self.teleportEntity(self.robots[i]['id'], robot_pose)
            self.robot_images[i] = robot_image
            self.robot_locations[i] = robot_loc

        # Unpause gazebo
        self.unpauseGazebo()

        # setup odometry thread to write out odometry values
        if self.allow_logging:
            self.log_lock.acquire()
            self.record_odometry = True
            self.log_file = open(log_file_name, 'w')
            self.log_file.write("odometry: \n")
            self.log_lock.release()

        # Save experiment start time
        self.experiment_start_time = rospy.get_time()
        self.experiment_max_duration = experiment_data['max_duration']

        # Setup interactions with user
        self.experiment_tutorial = experiment_data['is_tutorial']
        if self.experiment_tutorial:
            self.experiment_tutorial_duration = experiment_data['tutorial_duration']
            experiment_text = "#" + str(experiment_number + 1) + " The TUTORIAL has started! Use this tutorial to become comfortable with moving the person around."
        else:
            experiment_text = "#" + str(experiment_number + 1) + " The EXPERIMENT has started! Find the red ball as quickly as possible!"

        self.experiment_in_progress = True
        self.experiment_success = False
        self.pause_person_plugin(False)

        self.experiment_interface.startExperiment(experiment_text)

    def stopCurrentExperiment(self, success):

        self.experiment_in_progress = False
        self.pause_person_plugin(True)

        if not self.experiment_tutorial:
            current_time = rospy.get_time()
            time_diff = current_time - self.experiment_start_time
            reward_time = self.experiment_max_duration - time_diff

            # Send message to user
            if (success):
                reward = int(reward_time) + 50
                experiment_text = "You found the goal in " + str(time_diff) + " seconds and earned " + str(reward) + " points!"
            else:
                reward = 25
                experiment_text = "Oh no! It looks like you ran out of time with this experiment. We've awarded you " + str(reward) + " points."
        else:
            if (success):
                if self.experiments[self.experiment_number + 1]['is_tutorial']:
                    experiment_text = "Awesome! You found the goal! There are more TUTORIALS to go!"
                else:
                    experiment_text = "Awesome! You've finished the tutorials! Hit the enter key or press the continue button to proceed to the EXPERIMENTS!"
                reward = 50
            else:
                current_time = rospy.get_time()
                time_diff = current_time - self.experiment_start_time
                experiment_text = "Oh no! You were unable to find the ball quickly enough (" + str(time_diff) + " seconds). You need to find the ball in " + str(self.experiment_tutorial_duration) + " seconds. Let's try the tutorial again!"
                reward = 0

        experiment_text += " Hit the ENTER key or press Continue to try again!"
        self.experiment_interface.stopCurrentExperiment(reward, experiment_text)

        # setup odometry thread to stop writing out odometry values
        if self.allow_logging:
            self.log_lock.acquire()
            self.record_odometry = False
            self.log_file.write("success: " + str(success))
            self.log_file.close()
            self.log_lock.release()

        #teleport all the robots to correct positions
        self.pauseGazebo()
        for robot in self.robots:
            robot_pose = getPoseMsgFrom2dData(*robot['default_loc'], yaw=0)
            self.teleportEntity(robot['id'], robot_pose) 
        self.unpauseGazebo() 

    def odometryCallback(self, odom):

        if (not self.experiment_in_progress):
            return

        loc = [odom.pose.pose.position.x, odom.pose.pose.position.y]

        if not self.experiment_tutorial:
            # record position
            self.log_lock.acquire()
            self.log_file.write("  - [" + str(odom.pose.pose.position.x) + ", " + str(odom.pose.pose.position.y) + ", " + str(odom.header.stamp.to_sec()) + "]\n")
            self.log_lock.release()

        # check if the person is near a robot
        for i, robot in enumerate(self.robots):
            if i < len(self.robots_in_experiment):
                robot_loc = self.robot_locations[i]
                distance_sqr = (loc[0] - robot_loc[0]) * (loc[0] - robot_loc[0]) + (loc[1] - robot_loc[1]) * (loc[1] - robot_loc[1])
                if distance_sqr < 3.0 * 3.0:
                    self.robot_image_publisher.updateImage(robot['id'], self.robot_images[i])
                else:
                    self.robot_image_publisher.updateImage(robot['id'], self.image_none)

        # check if the person is close to the ball
        distance_sqr = (loc[0] - self.goal_location[0]) * (loc[0] - self.goal_location[0]) + (loc[1] - self.goal_location[1]) * (loc[1] - self.goal_location[1])
        if distance_sqr < 1.0 * 1.0:
            self.experiment_success = True

    def start(self):

        self.robot_image_publisher.start()

        r = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():

            time = rospy.get_time()

            self.experiment_lock_mutex.acquire()

            if self.start_next_experiment:
                self.startExperiment(self.data_directory + "/" + self.log_file_prefix + self.experiment_names[self.experiment_number], self.experiments[self.experiment_number], self.experiment_number)
                self.start_next_experiment = False
            elif self.experiment_in_progress and self.experiment_success:
                if self.experiment_tutorial:
                    if (time - self.experiment_start_time) > self.experiment_tutorial_duration: # too slow
                        self.stopCurrentExperiment(False)
                    else:
                        self.stopCurrentExperiment(True)
                        self.experiment_number = self.experiment_number + 1
                        self.allow_logging = True
                else: 
                    self.stopCurrentExperiment(True)
                    self.experiment_number = self.experiment_number + 1
                    self.allow_logging = True
                self.experiment_server_locked = self.experiment_number != len(self.experiments)
                if self.experiment_server_locked:
                    self.startResetTimer()
                else:
                    self.experiment_interface.stopExperiment()
            elif self.experiment_in_progress and (time - self.experiment_start_time) > self.experiment_max_duration:
                self.stopCurrentExperiment(False)
                if not self.experiment_tutorial:
                    self.experiment_number = self.experiment_number + 1
                    self.allow_logging = True
                else:
                    self.allow_logging = False
                self.experiment_server_locked = self.experiment_number != len(self.experiments)
                if self.experiment_server_locked:
                    self.startResetTimer()
                else:
                    self.experiment_interface.stopExperiment()

            #publish the experiment status
            self.experiment_interface.publishStatus()
            self.experiment_lock_mutex.release()

            r.sleep()

    def getGraphLocation(self, id):
        for entry in self.graph:
            if entry['id'] == id:
                return [entry['x'], entry['y']]
        raise IndexError, "Graph id %d does not exist in graph: %s"%(id, self.graph_file)

    def teleportEntity(self, entity, pose):
        set_state = SetModelStateRequest()
        set_state.model_state.model_name = entity
        set_state.model_state.pose = pose
        resp = self.teleport(set_state)
        if not resp.success:
            rospy.logerr(resp.status_message)

    def startResetTimer(self, time = 300.0):
        self.reset_timer_lock.acquire()
        if not self.reset_timer:
            self.reset_timer = threading.Timer(5.0, self.tickResetTimer, [time])
            self.reset_time_remaining = time
            self.reset_timer.start()
            rospy.loginfo("Started RESET timer (" + str(time) + ") for " + self.experiment_uid)
        else:
            rospy.logerr("Tried to start a timer when one exists")
        self.reset_timer_lock.release()

    def cancelResetTimer(self):
        self.reset_timer_lock.acquire()
        if self.reset_timer:
            rospy.loginfo("Cancelled RESET timer for " + self.experiment_uid)
            self.reset_timer.cancel()
            self.reset_timer = None
        else:
            rospy.logerr("Tried to cancel non-existant RESET timer")
        self.reset_timer_lock.release()

    def tickResetTimer(self, time_remaining):
        self.reset_timer_lock.acquire()
        if self.reset_timer: # check in case cancel timer has been called
            if time_remaining <= 0.0:
                self.reset_timer = None
                self.resetExperiment()
            else:
                time_remaining -= 5
                self.reset_time_remaining = time_remaining
                if self.paused:
                    self.experiment_interface.publishStatus()
                self.reset_timer = threading.Timer(5.0, self.tickResetTimer, [time_remaining])
                self.reset_timer.start()
        self.reset_timer_lock.release()

    def resetExperiment(self):
        self.experiment_lock_mutex.acquire()
        if self.experiment_server_locked:
            if self.experiment_in_progress:
                self.stopCurrentExperiment(False)
            self.reward = -1
            rospy.loginfo("RESET CALLED for user: " + self.experiment_uid)
            self.experiment_interface.resetExperiment()
            self.experiment_server_locked = False
            self.experiment_number = len(self.experiments)
            self.paused = False
            self.unpauseGazebo()
        self.experiment_lock_mutex.release()

    def startExperimentsForNewUser(self, name, email):
        self.experiment_lock_mutex.acquire()
        uid = id_generator()
        if not self.experiment_server_locked:
            self.experiment_names = ["tutorial_" + str(i) for i in range(len(self.tutorials))]
            self.experiments = copy.deepcopy(self.tutorials) 
            control_first = random.choice([True, False])
            if control_first:
                self.experiment_names.extend(["control_" + str(i) for i in range(len(self.control_experiments))])
                self.experiments.extend(self.control_experiments)
                self.experiment_names.extend(["test_" + str(i) for i in range(len(self.test_experiments))])
                self.experiments.extend(self.test_experiments)
            else:
                self.experiment_names.extend(["test_" + str(i) for i in range(len(self.test_experiments))])
                self.experiments.extend(self.test_experiments)
                self.experiment_names.extend(["control_" + str(i) for i in range(len(self.control_experiments))])
                self.experiments.extend(self.control_experiments)
            self.experiment_number = 0
            self.reward = 0

            self.experiment_uname = name
            self.experiment_uemail = email
            self.experiment_uid = uid
            self.num_experiments = len(self.experiments)

            rospy.loginfo("Starting experiments for user: " + uid)
            self.log_file_prefix = uid + "_"

            self.experiment_server_locked = True
            self.start_next_experiment = True
            success = True
        else:
            success = False
        self.experiment_lock_mutex.release()
        return success, uid

    def startNextExperiment(self):
        success = False
        self.experiment_lock_mutex.acquire()
        if not self.experiment_in_progress and self.experiment_server_locked:
            self.start_next_experiment = True
            self.cancelResetTimer()
            success = True
        self.experiment_lock_mutex.release()
        return success

    def pauseCurrentExperiment(self):
        success = False
        self.experiment_lock_mutex.acquire()
        if self.experiment_in_progress and self.experiment_server_locked:
            self.pauseGazebo()
            self.startResetTimer()
            self.paused = True
            success = True
        self.experiment_lock_mutex.release()
        return success

    def unpauseCurrentExperiment(self):
        success = False
        self.experiment_lock_mutex.acquire()
        if self.paused and self.experiment_in_progress and self.experiment_server_locked:
            self.unpauseGazebo()
            self.cancelResetTimer()
            self.paused = False
            success = True
        self.experiment_lock_mutex.release()
        return success

if __name__ == '__main__':
    try:
        controller = ExperimentController()
        controller.start()
    except rospy.ROSInterruptException:
        pass
