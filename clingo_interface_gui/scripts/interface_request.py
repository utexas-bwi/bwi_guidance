#! /usr/bin/env python

import roslib; roslib.load_manifest('clingo_interface_gui')
import rospy

# Brings in the SimpleActionClient
import actionlib

import clingo_interface_gui.msg
import sys

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('clingo_interface_gui', clingo_interface_gui.msg.ClingoInterfaceAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    command = clingo_interface_gui.msg.ClingoFluent(sys.argv[1], [sys.argv[2]])
    sense_fluent = clingo_interface_gui.msg.ClingoFluent()
    if command.op == "sense":
        sense_fluent = clingo_interface_gui.msg.ClingoFluent(sys.argv[2], [sys.argv[3]])

    goal = clingo_interface_gui.msg.ClingoInterfaceGoal(command, sense_fluent)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print "Result:", result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
