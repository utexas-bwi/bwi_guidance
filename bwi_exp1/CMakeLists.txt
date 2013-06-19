cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# find yaml lib definitions
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
include_directories(${YAML_CPP_INCLUDE_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(bwi_exp1
  src/libbwi_exp1/users.cpp
  src/libbwi_exp1/odometry.cpp
  src/libbwi_exp1/experiment.cpp
  )
target_link_libraries(bwi_exp1 ${YAML_CPP_LIBRARIES})

rosbuild_add_library(bwi_exp1_mdp
  src/mdp.cpp
)

rosbuild_add_executable(create_experiment src/create_experiment.cpp)
rosbuild_add_executable(robot_positioner src/nodes/robot_positioner.cpp)

#test exes
rosbuild_add_executable(test_mdp test/test_mdp.cpp)
target_link_libraries(test_mdp bwi_exp1_mdp)
