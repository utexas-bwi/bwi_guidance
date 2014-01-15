#!/bin/bash

export LD_LIBRARY_PATH=~/ros_libraries:${LD_LIBRARY_PATH}
export PATH=~/.local/bin:${PATH}
. ~/catkin_ws/devel/setup.bash
rosrun bwi_guidance_solver evaluate_qrr14 \
  --allow-robot-current \
  --mcts-params `rospack find bwi_guidance_solver`/config/mcts_params.json \
  "$@"
