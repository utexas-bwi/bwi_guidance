# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(bwi_msgs)
find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation)

add_message_files(
  FILES
  ExperimentStatus.msg
  ExperimentServerStatus.msg
)

add_service_files(
  FILES
  PositionRobot.srv
  UpdateExperiment.srv
  UpdateExperimentServer.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs
)

catkin_package(
    DEPENDS geometry_msgs
    CATKIN_DEPENDS message_runtime
)
