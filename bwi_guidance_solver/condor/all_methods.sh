#!/bin/bash

export LD_LIBRARY_PATH=~/ros_libraries:${LD_LIBRARY_PATH}
export PATH=~/.local/bin:${PATH}
. ~/catkin_ws/devel/setup.bash
rosrun bwi_guidance_solver evaluate_qrr14 --map-file `rospack find bwi_guidance`/maps/map3.yaml --graph-file `rospack find bwi_guidance`/maps/graph_map3.yaml --data-directory $1 --allow-robot-current --seed $2 --mcts-params `rospack find bwi_guidance_solver`/config/mcts_params.json --methods-file `rospack find bwi_guidance_solver`/config/all_methods.json
