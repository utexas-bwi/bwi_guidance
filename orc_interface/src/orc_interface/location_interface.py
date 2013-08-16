#!/usr/bin/env python

import rospy
import tf
from tf.transformations import euler_from_quaternion

import orc_interface

class LocationInterface:

    def __init__(self, robots):

        #TODO select only relevant targets here?
        self.robots = robots

        self.robot_locations = {}
        self.listener = tf.TransformListener()
        self.call_id_map = {}

    def invoke(self, call_id, target, args):
        self.call_id_map[call_id] = target
        
    def cancel(self, call_id):
        pass

    def check_status(self, call_id):
        robot_id = self.call_id_map[call_id]
        try:
            (trans, rot) = self.listener.lookupTransform(
                '/map', robot_id + '/base_footprint', rospy.Time(0)
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            fail_message = orc_interface.InterfaceMessage()
            fail_message.type = orc_interface.FAILURE
            fail_message.call_id = call_id
            fail_message.fail_type = 1
            fail_message.message = 'Unable to get location for ' + robot_id
            fail_message.causes = []
            return True, fail_message 
        (roll,pitch,yaw) = euler_from_quaternion(rot)
        ret_message = orc_interface.InterfaceMessage()
        ret_message.type = orc_interface.RETURN
        ret_message.call_id = call_id
        ret_message.value = [trans[0], trans[1], yaw]
        return True, ret_message
