#!/usr/bin/env python

import actionlib
from actionlib_msgs.msg import GoalStatus
import move_base_msgs.msg
import rospy
import tf

import orc_interface

class MoveInterface:

    def __init__(self, robots):

        #TODO select only relevant targets here?
        self.robots = robots

        self.robot_controllers = {}
        for robot in self.robots:
            rospy.loginfo('Requesting controller for ' + robot + '...')
            self.robot_controllers[robot] = actionlib.ActionClient(
                robot + '/move_base', 
                move_base_msgs.msg.MoveBaseAction
            )
            self.robot_controllers[robot].wait_for_server()
            rospy.loginfo('  Done...')


        self.call_id_map = {}
            
    def invoke(self, call_id, target, args):

        #Throw exception if target is not present, or args are malformed

        rospy.loginfo("MoveInterface: Moving %s to %s"%(target,str(args)))

        # Generate goal
        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.pose.position.x = args[0]
        goal.target_pose.pose.position.y = args[1]
        goal.target_pose.pose.position.z = 0
        q = tf.transformations.quaternion_from_euler(0, 0, args[2])
        goal.target_pose.pose.orientation.x = q[0] 
        goal.target_pose.pose.orientation.y = q[1] 
        goal.target_pose.pose.orientation.z = q[2] 
        goal.target_pose.pose.orientation.w = q[3] 

        gh = self.robot_controllers[target].send_goal(goal) 
        self.call_id_map[call_id] = gh
        
    def cancel(self, call_id):

        # Throw exception if call_id not in map
        gh = self.call_id_map[call_id]
        gh.cancel()

    def check_status(self, call_id):
        gh = self.call_id_map[call_id]
        status = gh.get_goal_status()
        if status in [GoalStatus.PENDING, GoalStatus.ACTIVE, 
                      GoalStatus.PREEMPTING, GoalStatus.RECALLING]:
            return False, None
        elif status in [GoalStatus.PREEMPTED, GoalStatus.ABORTED,
                        GoalStatus.REJECTED, GoalStatus.RECALLED, 
                        GoalStatus.LOST]:
            fail_message = orc_interface.InterfaceMessage()
            fail_message.type = orc_interface.FAILURE
            fail_message.call_id = call_id
            fail_message.fail_type = status
            fail_message.message = 'Could not complete the goal'
            fail_message.causes = []
            return True, fail_message
        elif status in [GoalStatus.SUCCEEDED]:
            ret_message = orc_interface.InterfaceMessage()
            ret_message.type = orc_interface.RETURN
            ret_message.call_id = call_id
            ret_message.value = 0 #TODO
            return True, ret_message

        #TODO throw exception here (should never be here)
