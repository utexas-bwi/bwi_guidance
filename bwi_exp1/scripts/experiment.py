#!/usr/bin/env python

import roslib; roslib.load_manifest('bwi_exp1')
import rospy

import yaml
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from bwi_msgs.srv import PositionRobot, PositionRobotRequest, UpdatePluginState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest

import cv, cv2, numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import threading
import math

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
        self.num_experiments = len(self.experiment['experiments'])

        self.person_id = self.experiment['person_id']
        self.robots = self.experiment['robots']
        self.ball_id = self.experiment['ball_id']

        # Read the arrows
        images_dir = roslib.packages.get_pkg_dir("bwi_web") + "/images"
        self.arrow = readImage(images_dir + "/Up.png")
        self.image_none = numpy.zeros((120,160,3), numpy.uint8)

        # Setup the Robot image publisher
        self.robot_image_publisher = RobotScreenPublisher()

        # Setup some experiment defaults
        self.robots_in_experiment = []
        self.robot_locations = []
        self.robot_images = []
        self.experiment_initialized = False

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

        # Get the person pause service
        rospy.loginfo("Waiting for service: " + self.person_id + "/update_state")
        rospy.wait_for_service(self.person_id + "/update_state")
        self.pause_person_plugin = rospy.ServiceProxy(self.person_id + "/update_state", UpdatePluginState)
        rospy.loginfo("Service acquired: " + self.person_id + "/update_state")

        for i, robot in enumerate(self.robots):
            robot_id = robot['id']
            self.robot_image_publisher.addRobot(robot_id)

        # Setup the odometry subscriber
        sub = rospy.Subscriber(self.person_id + '/odom', Odometry, self.odometryCallback)

        # Setup the experiment instructions publisher
        self.text_pub = rospy.Publisher('~user_text', String)

    def reset(self):
        self.startExperiment(0)

    def startExperiment(self, experiment_number):
        self.experiment_number = experiment_number
        experiment_data = self.experiment['experiments'][experiment_number]

        # Teleport person to correct place, however pause and unpause
        rospy.loginfo("Pausing person plugin")
        self.pause_person_plugin(True)
        start_x = experiment_data['start_x']
        start_y = experiment_data['start_y']
        start_yaw = experiment_data['start_yaw']
        start_pose = getPoseMsgFrom2dData(start_x, start_y, yaw=start_yaw)
        result = self.teleportEntity(self.person_id, start_pose)
        rospy.loginfo("Unpause person plugin")
        self.pause_person_plugin(False)

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
                    going_to = req.to_pt

                resp = self.position_robot(req)
                robot_loc = [resp.loc.x, resp.loc.y]
                robot_yaw = resp.yaw

                #get orientation of the arrow on the screen
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

        # Publish message to the user
        self.text_pub.publish("Experiment #" + str(experiment_number + 1) + " has started!");

        self.experiment_initialized = True

        # TODO Start recording odometry

        # Save experiment start time
        self.experiment_start_time = rospy.get_time()
        self.experiment_max_duration = experiment_data['max_duration']

    def stopCurrentExperiment(self, success):
        self.experiment_initialized = False
        # Send message to user
        if (success):
            self.text_pub.publish("It looks like you've found the goal. Let's proceed to the next experiment!")
        else:
            self.text_pub.publish("The experiment has timed out. Let's proceed to the next experiment.")
        
        # TODO Close the odometry write here 

    def odometryCallback(self, odom):

        if (not self.experiment_initialized):
            return

        loc = [odom.pose.pose.position.x, odom.pose.pose.position.y]

        # check if the person is near a robot
        for i, robot in enumerate(self.robots):
            if i < len(self.robots_in_experiment):
                robot_loc = self.robot_locations[i]
                distance_sqr = (loc[0] - robot_loc[0]) * (loc[0] - robot_loc[0]) + (loc[1] - robot_loc[1]) * (loc[1] - robot_loc[1])
                if distance_sqr < 3.0 * 3.0:
                    self.robot_image_publisher.updateImage(robot['id'], self.robot_images[i])
                else:
                    self.robot_image_publisher.updateImage(robot['id'], self.image_none)

    def start(self):
        self.robot_image_publisher.start()
        self.reset()
        rospy.spin()

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

if __name__ == '__main__':
    try:
        controller = ExperimentController()
        controller.start()
    except rospy.ROSInterruptException:
        pass
