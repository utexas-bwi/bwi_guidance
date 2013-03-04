cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(ROS_BUILD_TYPE Debug)
# set(CMAKE_VERBOSE_MAKEFILE on)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# find yaml and opencv lib definitions
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
include_directories(${YAML_CPP_INCLUDE_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

pkg_check_modules(OPENCV REQUIRED opencv)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

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

#main library
rosbuild_add_library(topological_mapper 
  src/libtopological_mapper/circle_provider.cpp 
  src/libtopological_mapper/map_inflator.cpp 
  src/libtopological_mapper/map_loader.cpp
  src/libtopological_mapper/directed_dfs.cpp
  src/libtopological_mapper/connected_components.cpp
  src/libtopological_mapper/voronoi_approximator.cpp
  src/libtopological_mapper/structures/point.cpp
  src/libtopological_mapper/structures/voronoi_point.cpp
  )

target_link_libraries(topological_mapper ${YAML_CPP_LIBRARIES})
target_link_libraries(topological_mapper ${OPENCV_LIBRARIES})
target_link_libraries(topological_mapper image_loader) #from map_server

#test binaries
rosbuild_add_executable(test_circle test/test_circle.cpp)
target_link_libraries(test_circle topological_mapper)

rosbuild_add_executable(test_map_loader test/test_map_loader.cpp)
target_link_libraries(test_map_loader topological_mapper)

rosbuild_add_executable(test_voronoi test/test_voronoi.cpp)
target_link_libraries(test_voronoi topological_mapper)

rosbuild_add_executable(test_dfs test/test_dfs.cpp)
target_link_libraries(test_dfs topological_mapper)

rosbuild_add_executable(test_graph test/test_graph.cpp)
target_link_libraries(test_graph topological_mapper)
