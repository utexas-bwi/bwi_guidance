cmake_minimum_required(VERSION 2.8.3)
project(bwi_guidance_solver)

## Find catkin and external packages
find_package(catkin REQUIRED COMPONENTS bwi_guidance bwi_mapper rl_pursuit)
find_package(Boost REQUIRED COMPONENTS serialization program_options)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    bwi_guidance_solver
  CATKIN_DEPENDS 
    bwi_guidance 
    bwi_mapper
)

###########
## Build ##
###########

add_definitions(-DVI_DEBUG)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(bwi_guidance_solver
  src/heuristic_solver.cpp
  src/person_model2.cpp
  src/person_estimator2.cpp
  src/structures.cpp
)
target_link_libraries(bwi_guidance_solver 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Declare a cpp executable
add_executable(test_vi 
  test/test_vi.cpp
)
target_link_libraries(test_vi 
  bwi_guidance_solver
)

add_executable(test_mdp2 
  test/test_mdp2.cpp
)
target_link_libraries(test_mdp2 
  bwi_guidance_solver
)
add_executable(generate_exp2 
  test/generate_exp2.cpp
)
target_link_libraries(generate_exp2 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(evaluate 
  src/evaluate.cpp
)
target_link_libraries(evaluate 
  bwi_guidance_solver
)
add_executable(metric_map2 
  src/metric_map2.cpp
)
target_link_libraries(metric_map2 
  bwi_guidance_solver
)

add_executable(vi_robot_positioner2
  src/nodes/vi_robot_positioner2.cpp
)
target_link_libraries(vi_robot_positioner2 
  bwi_guidance_solver
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS 
  bwi_guidance_solver
  test_mdp2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
