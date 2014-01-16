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

#add_definitions(-DVI_DEBUG)
#add_definitions(-DMCTS_DEBUG)
#add_definitions(-DUCT_DEBUG)
#add_definitions(-DEVALUATE_DEBUG)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(bwi_guidance_solver
  src/libbwi_guidance_solver/heuristic_solver_qrr14.cpp
  src/libbwi_guidance_solver/person_estimator_qrr14.cpp
  src/libbwi_guidance_solver/person_model_qrr14.cpp
  src/libbwi_guidance_solver/structures_qrr14.cpp
)
target_link_libraries(bwi_guidance_solver 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Declare a cpp executable
add_executable(test_vi 
  test/test_vi.cpp
)

add_executable(test_mdp_qrr14 
  test/test_mdp_qrr14.cpp
)
target_link_libraries(test_mdp_qrr14 
  bwi_guidance_solver
)
add_executable(generate_exp2_robots_qrr14 
  test/generate_exp2_robots_qrr14.cpp
)
target_link_libraries(generate_exp2_robots_qrr14
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(evaluate_qrr14 
  src/nodes/evaluate_qrr14.cpp
)
target_link_libraries(evaluate_qrr14
  bwi_guidance_solver
)
add_executable(metric_map2_qrr14 
  src/nodes/metric_map2_qrr14.cpp
)
target_link_libraries(metric_map2_qrr14 
  bwi_guidance_solver
)

add_executable(robot_positioner_qrr14
  src/nodes/robot_positioner_qrr14.cpp
)
target_link_libraries(robot_positioner_qrr14 
  bwi_guidance_solver
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS 
  bwi_guidance_solver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
