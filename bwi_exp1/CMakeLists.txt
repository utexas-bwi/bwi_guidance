# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(bwi_exp1)

find_package(catkin REQUIRED COMPONENTS bwi_msgs topological_mapper)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
    DEPENDS yaml-cpp
    CATKIN_DEPENDS bwi_msgs topological_mapper
    INCLUDE_DIRS include
    LIBRARIES bwi_exp1 bwi_exp1_mdp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS})

add_library(bwi_exp1
  src/libbwi_exp1/users.cpp
  src/libbwi_exp1/odometry.cpp
  src/libbwi_exp1/experiment.cpp
  )
target_link_libraries(bwi_exp1 
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

add_library(bwi_exp1_mdp
  src/mdp.cpp
)
target_link_libraries(bwi_exp1_mdp 
  ${catkin_LIBRARIES})

add_executable(create_experiment src/create_experiment.cpp)
target_link_libraries(create_experiment 
  ${catkin_LIBRARIES})
add_executable(robot_positioner src/nodes/robot_positioner.cpp)
target_link_libraries(robot_positioner 
  ${catkin_LIBRARIES})

#test the mdp
add_executable(test_mdp test/test_mdp.cpp)
target_link_libraries(test_mdp bwi_exp1_mdp)

#install targets
install(TARGETS
  bwi_exp1
  bwi_exp1_mdp
  create_experiment
  robot_positioner
  test_mdp
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

foreach(dir config data images include launch maps Media worlds www)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(PROGRAMS
  scripts/experiment.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
