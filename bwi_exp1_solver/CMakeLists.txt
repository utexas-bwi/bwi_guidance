cmake_minimum_required(VERSION 2.8.3)
project(bwi_exp1_solver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS bwi_exp1 topological_mapper)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bwi_exp1_mdp
  CATKIN_DEPENDS bwi_exp1 topological_mapper
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

## Declare a cpp library
add_library(bwi_exp1_mdp
  src/mdp.cpp
)
target_link_libraries(bwi_exp1_mdp 
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(test_mdp 
  test/test_mdp.cpp
)
target_link_libraries(test_mdp 
  bwi_exp1_mdp
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS 
  bwi_exp1_mdp
  test_mdp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
