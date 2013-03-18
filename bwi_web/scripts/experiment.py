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
#from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from bwi_msgs.srv import Teleport

def getPoseMsgFrom2dData(x, y, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0
    pose.orientation = Quaternion(*quaternion_from_euler(0,0,yaw))
    return pose

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

        # Get the person teleporter service
        rospy.wait_for_service(self.person_id + '/teleport')
        self.teleport_person = rospy.ServiceProxy(self.person_id + '/teleport', Teleport)

        # Get the robot teleporter service
        self.teleport_robot = [None] * len(self.robots)
        for i, robot in enumerate(self.robots):
            robot_id = robot['id']
            rospy.wait_for_service(robot_id + '/teleport')
            self.teleport_robot[i] = rospy.ServiceProxy(robot_id + '/teleport', Teleport)

        # Setup the odometry subscriber
        sub = rospy.Subscriber(self.person_id + '/odom', Odometry, self.odometryCallback)
        #self.text_pub = rospy.Publisher('/experiment/user_text', String)

    def reset(self):
        self.startExperiment(0)

    def startExperiment(self, experiment_number):
        self.experiment_number = experiment_number
        experiment_data = self.experiment['experiments'][experiment_number]

        # Teleport person to correct place
        graph_id = experiment_data['start_id']
        start_loc = self.getGraphLocation(graph_id) 
        start_yaw = experiment_data['start_yaw']
        start_pose = getPoseMsgFrom2dData(*start_loc, yaw=start_yaw)
        self.teleport_person(start_pose)

        # Teleport all the robots to their respective positions
        robots_in_experiment = experiment_data['robots']
        for i, robot in enumerate(self.robots):
            if i < len(robots_in_experiment):
                graph_id = robots_in_experiment[i]['id']
                robot_loc = self.getGraphLocation(graph_id) 
                robot_yaw = robots_in_experiment[i]['yaw']
            else:
                robot_loc = self.robots['default_loc']
                robot_yaw = 0
            robot_pose = getPoseMsgFrom2dData(*robot_loc, yaw=robot_yaw)
            self.teleport_robot[i](robot_pose)

    def odometryCallback(self, data):
        pass

    def start(self):
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
