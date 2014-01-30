#!/bin/bash

export LD_LIBRARY_PATH=~/ros_libraries:${LD_LIBRARY_PATH}
export PATH=~/.local/bin:${PATH}
. ~/catkin_ws/devel/setup.bash
rosrun bwi_guidance_solver evaluate_iros14 \
  --mcts-params `rospack find bwi_guidance_solver`/config/iros/mcts_params.json \
  "$@"
