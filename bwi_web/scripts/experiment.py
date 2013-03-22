#!/usr/bin/env python

# Read yaml file corresponding to graph
# Read experiment file
# Publish out text data corresponding to state of the experiment
# Teleport person and robots around
# Grab odometry data
# How do we want the robot to communicate with the human??
# have a service to aquire a lock onto this experiment

import roslib; roslib.load_manifest('bwi_web')
import rospy

import yaml
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from bwi_msgs.srv import Teleport

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
    image_from_file = cv2.imread(location)
    height, width, channels = image_from_file.shape
    height_ratio = 119.0 / height
    width_ratio = 159.0 / width
    min_ratio = min(height_ratio, width_ratio)

    resized_image = cv2.resize(image_from_file, (0,0), fx=min_ratio, fy=min_ratio)
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
        self.arrow_up = readImage(images_dir + "/Up.png")
        self.arrow_down = readImage(images_dir + "/Down.png")
        self.arrow_left = readImage(images_dir + "/Left.png")
        self.arrow_right = readImage(images_dir + "/Right.png")
        self.arrow_none = numpy.zeros((120,160,3), numpy.uint8)

        # Setup the Robot image publisher
        self.robot_image_publisher = RobotScreenPublisher()

        # Setup some experiment defaults
        self.robots_in_experiment = []
        self.robot_locations = []
        self.robot_images = []

        rospy.loginfo("Waiting for service: %s"%self.person_id+'/teleport')
        rospy.wait_for_service(self.person_id + '/teleport')
        self.teleport_person = rospy.ServiceProxy(self.person_id + '/teleport', Teleport)
        rospy.loginfo("Service acquired: %s"%self.person_id+'/teleport')

        # Get the robot teleporter service
        self.teleport_robot = [None] * len(self.robots)
        for i, robot in enumerate(self.robots):
            robot_id = robot['id']
            rospy.loginfo("Waiting for service: %s"%robot_id+'/teleport')
            rospy.wait_for_service(robot_id + '/teleport')
            self.teleport_robot[i] = rospy.ServiceProxy(robot_id + '/teleport', Teleport)
            rospy.loginfo("Service acquired: %s"%robot_id+'/teleport')
            self.robot_image_publisher.addRobot(robot_id)

        # Get the goal teleporter service
        rospy.loginfo("Waiting for service: %s"%self.ball_id+'/teleport')
        rospy.wait_for_service(self.ball_id + '/teleport')
        self.teleport_ball = rospy.ServiceProxy(self.ball_id + '/teleport', Teleport)
        rospy.loginfo("Service acquired: %s"%self.ball_id+'/teleport')

        # Setup the odometry subscriber
        sub = rospy.Subscriber(self.person_id + '/odom', Odometry, self.odometryCallback)

        # Setup the experiment instructions publisher
        self.text_pub = rospy.Publisher('~user_text', String)

    def reset(self):
        self.startExperiment(0)

    def startExperiment(self, experiment_number):
        self.experiment_number = experiment_number
        experiment_data = self.experiment['experiments'][experiment_number]

        # Teleport person to correct place
        start_x = experiment_data['start_x']
        start_y = experiment_data['start_y']
        start_yaw = experiment_data['start_yaw']
        start_pose = getPoseMsgFrom2dData(start_x, start_y, yaw=start_yaw)
        self.teleport_person(start_pose)

        # Teleport the goal ball to its correct position
        ball_x = experiment_data['ball_x']
        ball_y = experiment_data['ball_y']
        ball_pose = getPoseMsgFrom2dData(ball_x, ball_y, 0)
        self.teleport_ball(ball_pose)
        self.goal_location = [ball_x, ball_y]

        # compute location/orientation from path?
        # will require saving the 2 parameters in the graph itself

        # Teleport all the robots to their respective positions
        self.path = experiment_data['path']
        self.robots_in_experiment = [{'id': p['id'], 'path_position': i} for i, p in enumerate(self.path) if p['robot']]
        print 
        self.robot_locations = [None] * len(self.robots)
        self.robot_images = [None] * len(self.robots)
        for i, robot in enumerate(self.robots):
            if i < len(self.robots_in_experiment):
                graph_id = self.robots_in_experiment[i]['id']
                robot_loc = self.getGraphLocation(graph_id)
                # compute yaw
                path_position = self.robots_in_experiment[i]['path_position']
                if path_position != 0:
                    arriving_from = self.getGraphLocation(self.path[path_position - 1]['id'])
                else:
                    arriving_from = [start_x, start_y]
                robot_yaw = math.atan2(robot_loc[1] - arriving_from[1], robot_loc[0] - arriving_from[0])
                # TODO this is not perfect when going straight
                robot_loc[0] = robot_loc[0] + 0.25 * math.cos(robot_yaw)
                robot_loc[1] = robot_loc[1] + 0.25 * math.sin(robot_yaw)
                # compute image
                if path_position != len(self.path) - 1:
                    going_to = self.getGraphLocation(self.path[path_position + 1]['id'])
                else:
                    going_to = [ball_x, ball_y]
                destination_yaw = math.atan2(going_to[1] - robot_loc[1], going_to[0] - robot_loc[0])
                change_in_yaw = destination_yaw - robot_yaw
                #normalize angle
                change_in_yaw = math.atan2(math.sin(change_in_yaw), math.cos(change_in_yaw))
                if change_in_yaw > 0.75 * math.pi or change_in_yaw <= -0.75 * math.pi:
                    robot_image = self.arrow_left
                elif change_in_yaw > 0.25 * math.pi and change_in_yaw <= 0.75 * math.pi:
                    robot_image = self.arrow_up
                elif change_in_yaw > -0.25 * math.pi and change_in_yaw <= 0.25 * math.pi:
                    robot_image = self.arrow_right
                else:
                    robot_image = self.arrow_down
            else:
                robot_loc = self.robots[i]['default_loc']
                robot_yaw = 0
                robot_image = self.arrow_none
            robot_pose = getPoseMsgFrom2dData(*robot_loc, yaw=robot_yaw)
            self.teleport_robot[i](robot_pose)
            self.robot_images[i] = robot_image
            self.robot_locations[i] = robot_loc

        # Publish message to the user
        self.text_pub.publish("Experiment #" + str(experiment_number + 1) + " has started!");

        # TODO Start recording odometry

        # Save experiment start time
        self.experiment_start_time = rospy.get_time()
        self.experiment_max_duration = experiment_data['max_duration']

    def stopCurrentExperiment(self, success):
        # Send message to user
        if (success):
            self.text_pub.publish("It looks like you've found the goal. Let's proceed to the next experiment!")
        else:
            self.text_pub.publish("The experiment has timed out. Let's proceed to the next experiment.")
        
        # TODO Close the odometry write here, plus say if this run was successful 

    def odometryCallback(self, odom):

        loc = [odom.pose.pose.position.x, odom.pose.pose.position.y]

        # check if the person is near a robot
        for i, robot in enumerate(self.robots):
            if i < len(self.robots_in_experiment):
                graph_id = self.robots_in_experiment[i]['id']
                robot_loc = self.getGraphLocation(graph_id)
                distance_sqr = (loc[0] - robot_loc[0]) * (loc[0] - robot_loc[0]) + (loc[1] - robot_loc[1]) * (loc[1] - robot_loc[1])
                if distance_sqr < 3.0 * 3.0:
                    self.robot_image_publisher.updateImage(robot['id'], self.robot_images[i])
                else:
                    self.robot_image_publisher.updateImage(robot['id'], self.arrow_none)

    def start(self):
        self.robot_image_publisher.start()
        self.reset()
        rospy.spin()

    def getGraphLocation(self, id):
        for entry in self.graph:
            if entry['id'] == id:
                return [entry['x'], entry['y']]
        raise IndexError, "Graph id %d does not exist in graph: %s"%(id, self.graph_file)

if __name__ == '__main__':
    try:
        controller = ExperimentController()
        controller.start()
    except rospy.ROSInterruptException:
        pass
