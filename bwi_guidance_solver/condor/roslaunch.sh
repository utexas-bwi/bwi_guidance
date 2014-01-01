#!/bin/bash

export LD_LIBRARY_PATH=~/ros_libraries:${LD_LIBRARY_PATH}
export PATH=~/.local/bin:${PATH}
. ~/catkin_ws/devel/setup.bash
roslaunch "$@"
