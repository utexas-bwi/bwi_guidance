# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(topological_mapper)

## Find catkin and external packages
find_package(catkin REQUIRED 
  COMPONENTS 
    map_server
    nav_msgs 
)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  DEPENDS 
    opencv2 
    yaml-cpp
  CATKIN_DEPENDS 
    map_server
    nav_msgs 
  INCLUDE_DIRS 
    include
  LIBRARIES 
    topological_mapper
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#main library
add_library(topological_mapper 
  src/libtopological_mapper/circle_provider.cpp 
  src/libtopological_mapper/map_inflator.cpp 
  src/libtopological_mapper/map_loader.cpp
  src/libtopological_mapper/map_utils.cpp
  src/libtopological_mapper/directed_dfs.cpp
  src/libtopological_mapper/connected_components.cpp
  src/libtopological_mapper/voronoi_approximator.cpp
  src/libtopological_mapper/topological_mapper.cpp
  src/libtopological_mapper/graph.cpp
  src/libtopological_mapper/point_utils.cpp
  src/libtopological_mapper/structures/point.cpp
  src/libtopological_mapper/structures/voronoi_point.cpp
)

target_link_libraries(topological_mapper 
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

#binaries
add_executable(generate_graph src/nodes/generate_graph.cpp)
target_link_libraries(generate_graph topological_mapper)

add_executable(view_graph src/nodes/view_graph.cpp)
target_link_libraries(view_graph topological_mapper)

add_executable(prepare_graph src/nodes/prepare_graph.cpp)
target_link_libraries(prepare_graph topological_mapper)

#test binaries
add_executable(test_circle test/test_circle.cpp)
target_link_libraries(test_circle topological_mapper)

add_executable(test_map_loader test/test_map_loader.cpp)
target_link_libraries(test_map_loader topological_mapper)

add_executable(test_voronoi test/test_voronoi.cpp)
target_link_libraries(test_voronoi topological_mapper)

add_executable(test_dfs test/test_dfs.cpp)
target_link_libraries(test_dfs topological_mapper)

add_executable(test_graph test/test_graph.cpp)
target_link_libraries(test_graph topological_mapper)

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)

install(TARGETS
  topological_mapper
  generate_graph
  view_graph
  prepare_graph
  test_circle
  test_map_loader
  test_voronoi
  test_dfs
  test_graph
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

