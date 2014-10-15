cmake_minimum_required(VERSION 2.8.3)
project(bwi_guidance_solver)

## Find catkin and external packages
find_package(catkin REQUIRED COMPONENTS bwi_guidance bwi_mapper bwi_rl pluginlib)
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
    bwi_rl
    pluginlib
)

###########
## Build ##
###########

#add_definitions(-DVI_DEBUG)
## add_definitions(-DMCTS_DEBUG)
add_definitions(-DMCTS_VALUE_DEBUG)
add_definitions(-DUCT_DEBUG)
add_definitions(-DEVALUATE_DEBUG)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(bwi_guidance_solver
  src/libbwi_guidance_solver/common.cpp
  src/libbwi_guidance_solver/irm/solver.cpp
  src/libbwi_guidance_solver/irm/heuristic_solver.cpp
  src/libbwi_guidance_solver/irm/person_estimator.cpp
  src/libbwi_guidance_solver/irm/person_model.cpp
  src/libbwi_guidance_solver/irm/structures.cpp
  src/libbwi_guidance_solver/irm/vi_solver.cpp
  src/libbwi_guidance_solver/irm/mcts_solver.cpp
  src/libbwi_guidance_solver/irm/domain.cpp
  src/libbwi_guidance_solver/mrn/heuristic_solver.cpp
  src/libbwi_guidance_solver/mrn/person_model.cpp
  src/libbwi_guidance_solver/mrn/structures.cpp
  src/libbwi_guidance_solver/mrn/domain.cpp
)
target_link_libraries(bwi_guidance_solver 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS 
  bwi_guidance_solver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
