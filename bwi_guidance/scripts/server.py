#!/usr/bin/env python

import rospy

from bwi_guidance import ExperimentServer

if __name__ == '__main__':
    try:
        server = ExperimentServer()
        server.start()
    except rospy.ROSInterruptException:
        pass
