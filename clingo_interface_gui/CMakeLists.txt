# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(clingo_interface_gui)
find_package(catkin REQUIRED COMPONENTS clingo_interface geometry_msgs nav_msgs tf actionlib actionlib_msgs)

add_message_files(
  FILES
  ClingoFluent.msg
)

add_action_files(
  FILES
  ClingoInterface.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  DEPENDS qt4
  CATKIN_DEPENDS actionlib actionlib_msgs clingo_interface geometry_msgs nav_msgs tf
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

##############################################################################
# QT Stuff
##############################################################################

# We are mixing boost and qt signals
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/clingo_interface_gui/*.hpp)

qt4_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt4_wrap_cpp(QT_MOC_HPP ${QT_MOC})

#To locate generate ui headers
include_directories(${CMAKE_CURRENT_BINARY_DIR})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(clingo_interface_gui ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(clingo_interface_gui 
  ${catkin_LIBRARIES}
  ${QT_QTCORE_LIBRARY} 
  ${QT_QTGUI_LIBRARY}
)

