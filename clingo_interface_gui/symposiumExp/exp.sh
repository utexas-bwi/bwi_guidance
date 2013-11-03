#!/bin/bash

for i in {1..20} 
do
  rosservice call /gazebo/set_model_state "{model_state: {model_name: 'segbot', pose: {position: {x: 22, y: 11.5}, orientation: {w: 1.0}}}}"
  cp query.in query
  cp initial.in initial
  sleep 1

  ./planner_cost.py nav926 initial query
  sleep 1
done
