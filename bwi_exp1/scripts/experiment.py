#!/usr/bin/env python

import roslib; roslib.load_manifest('bwi_exp1')
import rospy

import yaml
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from bwi_msgs.msg import ExperimentStatus
from bwi_msgs.srv import PositionRobot, PositionRobotRequest, UpdatePluginState, AcquireExperimentLock, AcquireExperimentLockResponse
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

class ExperimentTextPublisher(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.experiment_text = ""
        self.experiment_text_lock = threading.Lock()
        self.experiment_text_publisher = rospy.Publisher('~user_text', String)

    def updateText(self, text):
        self.experiment_text_lock.acquire()
        self.experiment_text = text
        self.experiment_text_lock.release()

    def run(self):
        r = rospy.Rate(10) # 10hz
        try:
            while not rospy.is_shutdown():
                self.experiment_text_publisher.publish(self.experiment_text)
                r.sleep()
        except rospy.ROSInterruptException:
            pass

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

        # Read the arrows
        images_dir = roslib.packages.get_pkg_dir("bwi_web") + "/images"
        self.arrow = readImage(images_dir + "/Up.png")
        self.image_none = numpy.zeros((120,160,3), numpy.uint8)

        # Setup the Robot image publisher
        self.robot_image_publisher = RobotScreenPublisher()

        # Setup the experiment text publisher
        self.experiment_text_publisher = ExperimentTextPublisher()

        # Setup some experiment defaults
        self.robots_in_experiment = []
        self.robot_locations = []
        self.robot_images = []
        self.experiment_initialized = False
        self.experiment_success = False

        self.experiment_lock = False
        self.experiment_lock_mutex = threading.Lock()
        self.experiment_uid = ""
        self.reward = 0
        self.experiment_lock_service_server = rospy.Service('~experiment_lock', AcquireExperimentLock, self.acquireExperimentLock)
        self.next_experiment_service_server = rospy.Service('~start_next_experiment', Empty, self.startNextExperiment)
        self.experiment_status_publisher = rospy.Publisher('~experiment_status', ExperimentStatus)
        self.start_next_experiment = False

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
        self.pause_gazebo = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        rospy.loginfo("Service acquired: /gazebo/pause_physics")

        rospy.loginfo("Waiting for service: /gazebo/unpause_physics")
        rospy.wait_for_service("/gazebo/unpause_physics")
        self.unpause_gazebo = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        rospy.loginfo("Service acquired: /gazebo/unpause_physics")

        # Get the person pause service
        rospy.loginfo("Waiting for service: " + self.person_id + "/update_state")
        rospy.wait_for_service(self.person_id + "/update_state")
        self.pause_person_plugin = rospy.ServiceProxy(self.person_id + "/update_state", UpdatePluginState)
        rospy.loginfo("Service acquired: " + self.person_id + "/update_state")
        self.pause_person_plugin(True)

        for i, robot in enumerate(self.robots):
            robot_id = robot['id']
            self.robot_image_publisher.addRobot(robot_id)

        # Setup the odometry subscriber
        sub = rospy.Subscriber(self.person_id + '/odom', Odometry, self.odometryCallback)

        # Log file lock
        self.log_lock = threading.Lock()

    def startExperiment(self, log_file_name, experiment_data, experiment_number):

        # Pause gazebo during teleportation
        self.pause_gazebo() # time also stops

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

                #get position of robot from service
                req = PositionRobotRequest()
                req.from_id = -1
                req.to_id = -1
                req.at_id = self.robots_in_experiment[i]['id']
                if path_position != 0:
                    req.from_id = self.path[path_position - 1]['id']
                else:
                    req.from_pt.x = start_x
                    req.from_pt.y = start_y
                    req.from_pt.z = 0
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

                #get orientation of the arrow on the screen
                #check if the next point is too close
                destination_distance = math.sqrt((going_to[1] - robot_loc[1])*(going_to[1] - robot_loc[1]) + (going_to[0] - robot_loc[0])*(going_to[0] - robot_loc[0]))
                path_position = path_position + 1
                while destination_distance < 1.25:
                    if path_position != len(self.path) - 1:
                        going_to = self.getGraphLocation(self.path[path_position + 1]['id'])
                    else:
                        going_to = [ball_x, ball_y]
                    destination_distance = math.sqrt((going_to[1] - robot_loc[1])*(going_to[1] - robot_loc[1]) + (going_to[0] - robot_loc[0])*(going_to[0] - robot_loc[0]))
                    path_position = path_position + 1

                destination_yaw = math.atan2(going_to[1] - robot_loc[1], going_to[0] - robot_loc[0])
                change_in_yaw = destination_yaw - robot_yaw
                change_in_yaw = math.atan2(math.sin(change_in_yaw), math.cos(change_in_yaw)) #normalize angle
                robot_image = produceDirectedArrow(self.arrow, change_in_yaw)

            elif i - len(self.robots_in_experiment) < len(self.extra_robots): #one of the extra robots
                robot_loc = [self.extra_robots[i - len(self.robots_in_experiment)]['loc_x'], self.extra_robots[i - len(self.robots_in_experiment)]['loc_y']]
                robot_yaw = self.extra_robots[i - len(self.robots_in_experiment)]['yaw']
                robot_image = self.image_none

            else: #robot not in use in this experiment
                robot_loc = self.robots[i]['default_loc']
                robot_yaw = 0
                robot_image = self.image_none

            #perform the actual teleport and image update
            robot_pose = getPoseMsgFrom2dData(*robot_loc, yaw=robot_yaw)
            result = self.teleportEntity(self.robots[i]['id'], robot_pose)
            self.robot_images[i] = robot_image
            self.robot_locations[i] = robot_loc

        # Unpause gazebo
        self.unpause_gazebo()

        # Publish message to the user
        self.experiment_tutorial = experiment_data['is_tutorial']
        if self.experiment_tutorial:
            self.experiment_tutorial_duration = experiment_data['tutorial_duration']
            self.experiment_text_publisher.updateText("#" + str(experiment_number + 1) + " The tutorial has started! Use this tutorial to become comfortable with moving the person around to find the red ball.")
        else:
            self.experiment_text_publisher.updateText("#" + str(experiment_number + 1) + " The experiment has started! Find the red ball as quickly as possible!")

        # setup odometry thread to write out odometry values
        if not self.experiment_tutorial:
            self.log_lock.acquire()
            self.record_odometry = True
            self.log_file = open(log_file_name, 'w')
            self.log_file.write("name: " + self.experiment_uname + "\n")  
            self.log_file.write("email: " + self.experiment_uemail + "\n")  
            self.log_file.write("odometry: \n")
            self.log_lock.release()

        # Save experiment start time
        self.experiment_start_time = rospy.get_time()
        self.experiment_max_duration = experiment_data['max_duration']

        self.experiment_initialized = True
        self.experiment_success = False
        self.pause_person_plugin(False)

    def stopCurrentExperiment(self, success):

        self.experiment_initialized = False
        self.pause_person_plugin(True)

        if not self.experiment_tutorial:
            current_time = rospy.get_time()
            time_diff = current_time - self.experiment_start_time
            reward_time = self.experiment_max_duration - time_diff

            # Send message to user
            if (success):
                reward = int(reward_time) + 50
                self.experiment_text_publisher.updateText("You found the goal in " + str(time_diff) + " seconds and earned " + str(reward) + " points! Hit the enter key or press the continue button to proceed!")
            else:
                reward = 25
                self.experiment_text_publisher.updateText("Oh no! It looks like you ran out of time with this experiment. We've awarded you " + str(reward) + " points. Hit the enter key or press the continue button to proceed!")
            self.reward = self.reward + reward
            
            # setup odometry thread to stop writing out odometry values
            self.log_lock.acquire()
            self.record_odometry = False
            self.log_file.write("success: " + str(success))
            self.log_file.close()
            self.log_lock.release()
        else:
            if (success):
                if self.experiments[self.experiment_number + 1]['is_tutorial']:
                    self.experiment_text_publisher.updateText("Awesome! You found the goal! Hit the enter key or press the continue button to proceed to the next tutorial!")
                else:
                    self.experiment_text_publisher.updateText("Awesome! That's it for the tutorial! Hit the enter key or press the continue button to proceed to the real experiments!")
            else:
                current_time = rospy.get_time()
                time_diff = current_time - self.experiment_start_time
                self.experiment_text_publisher.updateText("Oh no! You were unable to find the ball quickly enough (" + str(time_diff) + " seconds). You need to find the ball in " + str(self.experiment_tutorial_duration) + " seconds. Hit the enter key or press the continue button to try the tutorial again!") 

    def odometryCallback(self, odom):

        if (not self.experiment_initialized):
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
                #print i, robot_loc, loc
                distance_sqr = (loc[0] - robot_loc[0]) * (loc[0] - robot_loc[0]) + (loc[1] - robot_loc[1]) * (loc[1] - robot_loc[1])
                if distance_sqr < 3.0 * 3.0:
                    self.robot_image_publisher.updateImage(robot['id'], self.robot_images[i])
                else:
                    self.robot_image_publisher.updateImage(robot['id'], self.image_none)

        # check if the person is close to the ball
        distance_sqr = (loc[0] - self.goal_location[0]) * (loc[0] - self.goal_location[0]) + (loc[1] - self.goal_location[1]) * (loc[1] - self.goal_location[1])
        if distance_sqr < 1.0 * 1.0:
            self.experiment_success = True

    def startExperimentsForNewUser(self, uid, name, email):
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

        rospy.loginfo("Starting experiments for user: " + uid)
        self.experiment_text_publisher.updateText("The experiment will start in 5 seconds!")
        self.log_file_prefix = uid + "_"

        self.start_next_experiment = True

    def start(self):

        self.robot_image_publisher.start()
        self.experiment_text_publisher.start()

        r = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():

            time = rospy.get_time()

            self.experiment_lock_mutex.acquire()

            if self.start_next_experiment:
                self.startExperiment(self.data_directory + "/" + self.log_file_prefix + self.experiment_names[self.experiment_number], self.experiments[self.experiment_number], self.experiment_number)
                self.start_next_experiment = False
            elif self.experiment_initialized and self.experiment_success:
                if self.experiment_tutorial:
                    if (time - self.experiment_start_time) > self.experiment_tutorial_duration: # too slow
                        self.stopCurrentExperiment(False)
                    else:
                        self.stopCurrentExperiment(True)
                        self.experiment_number = self.experiment_number + 1
                else: 
                    self.stopCurrentExperiment(True)
                    self.experiment_number = self.experiment_number + 1
                self.experiment_lock = self.experiment_number != len(self.experiments)
            elif self.experiment_initialized and (time - self.experiment_start_time) > self.experiment_max_duration:
                self.stopCurrentExperiment(False)
                if not self.experiment_tutorial:
                    self.experiment_number = self.experiment_number + 1
                self.experiment_lock = self.experiment_number != len(self.experiments)

            #publish the experiment status
            status = ExperimentStatus()
            status.locked = self.experiment_lock
            status.uid = self.experiment_uid
            status.reward = self.reward
            status.experiment_in_progress = self.experiment_initialized
            self.experiment_status_publisher.publish(status)
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

    def acquireExperimentLock(self, req):
        resp = AcquireExperimentLockResponse();
        self.experiment_lock_mutex.acquire()
        if not self.experiment_lock:
            self.experiment_lock = True
            resp.uid = id_generator()
            self.startExperimentsForNewUser(resp.uid, req.name, req.email)
            resp.result = True
        else:
            resp.result = False
        self.experiment_lock_mutex.release()
        return resp

    def startNextExperiment(self, req):
        self.experiment_lock_mutex.acquire()
        self.start_next_experiment = True
        self.experiment_lock_mutex.release()

if __name__ == '__main__':
    try:
        controller = ExperimentController()
        controller.start()
    except rospy.ROSInterruptException:
        pass
