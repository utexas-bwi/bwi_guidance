# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(bwi_guidance)

find_package(catkin REQUIRED COMPONENTS bwi_guidance_msgs cv_bridge roslib bwi_mapper)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  DEPENDS yaml-cpp opencv
  CATKIN_DEPENDS bwi_guidance_msgs cv_bridge roslib bwi_mapper
  INCLUDE_DIRS include
  LIBRARIES bwi_guidance
)

catkin_python_setup()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS})

## Declare a cpp library
add_library(bwi_guidance
  src/libbwi_guidance/users.cpp
  src/libbwi_guidance/odometry.cpp
  src/libbwi_guidance/experiment.cpp
  src/libbwi_guidance/robots.cpp
  src/libbwi_guidance/base_robot_positioner.cpp
  src/libbwi_guidance/robot_screen_publisher.cpp
  )
target_link_libraries(bwi_guidance 
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES})
add_dependencies(bwi_guidance bwi_guidance_msgs_gencpp)

## Declare a cpp executable
add_executable(create_experiment src/create_experiment.cpp)
target_link_libraries(create_experiment 
  ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(robot_positioner src/nodes/robot_positioner.cpp)
target_link_libraries(robot_positioner 
  bwi_guidance
  ${catkin_LIBRARIES})

add_executable(test_robot_positioner test/test_robot_positioner.cpp)
target_link_libraries(test_robot_positioner 
  bwi_guidance
  ${catkin_LIBRARIES})

add_executable(test_screen_publisher test/test_screen_publisher.cpp)
target_link_libraries(test_screen_publisher 
  bwi_guidance
  ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## install(PROGRAMS
##   scripts/experiment.py
##   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS
  bwi_guidance
  create_experiment
  ## robot_positioner
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
foreach(dir config data images include launch maps Media worlds www)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

