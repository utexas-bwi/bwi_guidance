#!/bin/bash

mkdir -p output
for i in {1..30} 
do
  rosservice call /gazebo/set_model_state "{model_state: {model_name: 'segbot', pose: {position: {x: -6.0, y: 5}, orientation: {w: 1.0}}}}"
  cp initial.in initial
  cp query.asp.in query.asp
  sleep 1

  ./planner_cost.py nav.asp initial query.asp
  cp result output/result.${i}
  sleep 1
done
