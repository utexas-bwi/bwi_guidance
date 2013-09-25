cmake_minimum_required(VERSION 2.8.3)
project(bwi_exp1_solver)

## Find catkin and external packages
find_package(catkin REQUIRED COMPONENTS bwi_exp1 topological_mapper)
find_package(Boost REQUIRED COMPONENTS serialization program_options)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    bwi_exp1_solver
  CATKIN_DEPENDS 
    bwi_exp1 
    topological_mapper
)

###########
## Build ##
###########

## add_definitions(-DVI_DEBUG)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(bwi_exp1_solver
  src/heuristic_solver.cpp
  src/person_model.cpp
  src/person_model2.cpp
  src/person_estimator.cpp
  src/person_estimator2.cpp
  src/structures.cpp
)
target_link_libraries(bwi_exp1_solver 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Declare a cpp executable
add_executable(test_vi 
  test/test_vi.cpp
)
target_link_libraries(test_vi 
  bwi_exp1_solver
)
add_executable(test_mdp 
  test/test_mdp.cpp
)
target_link_libraries(test_mdp 
  bwi_exp1_solver
)
add_executable(test_mdp2 
  test/test_mdp2.cpp
)
target_link_libraries(test_mdp2 
  bwi_exp1_solver
)
add_executable(evaluate 
  src/evaluate.cpp
)
target_link_libraries(evaluate 
  bwi_exp1_solver
)

add_executable(vi_robot_positioner
  src/nodes/vi_robot_positioner.cpp
)
target_link_libraries(vi_robot_positioner 
  bwi_exp1_solver
  ${catkin_LIBRARIES}
)

add_executable(vi_robot_positioner2
  src/nodes/vi_robot_positioner2.cpp
)
target_link_libraries(vi_robot_positioner2 
  bwi_exp1_solver
  ${catkin_LIBRARIES}
)

add_executable(heuristic_robot_positioner
  src/nodes/heuristic_robot_positioner.cpp
)
target_link_libraries(heuristic_robot_positioner 
  bwi_exp1_solver
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS 
  bwi_exp1_solver
  test_mdp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
