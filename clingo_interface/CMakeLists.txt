# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(clingo_interface)
find_package(catkin REQUIRED COMPONENTS roscpp topological_mapper tf nav_msgs)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
    DEPENDS opencv2 yaml-cpp
    CATKIN_DEPENDS roscpp topological_mapper tf nav_msgs
    INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  )

add_executable(cc_test src/cc_test.cpp)
target_link_libraries(cc_test ${OpenCV_LIBRARIES})

add_executable(gazebo_handler src/gazebo_handler.cpp)
target_link_libraries(gazebo_handler 
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_executable(test_door_detector src/test_door_detector.cpp)
target_link_libraries(test_door_detector 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

add_executable(mark_locations src/mark_locations.cpp)
target_link_libraries(mark_locations 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

add_executable(mark_doors src/mark_doors.cpp)
target_link_libraries(mark_doors 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

install(TARGETS
  cc_test
  test_door_detector
  mark_doors
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
  include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

