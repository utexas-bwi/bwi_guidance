# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(bwi_guidance_msgs)
find_package(catkin REQUIRED COMPONENTS actionlib_msgs geometry_msgs message_generation)

add_message_files(
  FILES
  RobotInfo.msg
  RobotInfoArray.msg
  ExperimentStatus.msg
  ExperimentServerStatus.msg
)

add_service_files(
  FILES
  UpdateExperiment.srv
  UpdateExperimentServer.srv
)

add_action_files(
  DIRECTORY action
  FILES MultiRobotNavigation.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime
)
