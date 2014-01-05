#!/bin/bash

export LD_LIBRARY_PATH=~/ros_libraries:${LD_LIBRARY_PATH}
export PATH=~/.local/bin:${PATH}
. ~/catkin_ws/devel/setup.bash
rosrun bwi_guidance_solver evaluate_qrr14 \
  --map-file `rospack find bwi_guidance`/maps/map3.yaml \
  --graph-file `rospack find bwi_guidance`/maps/graph_map3.yaml \
  --data-directory /projects/agents1/piyushk/ \
  --allow-robot-current \
  --mcts-params `rospack find bwi_guidance_solver`/config/mcts_params.json \
  "$@"
