/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/bwi_exp1_visualizer/qnode.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace bwi_exp1_visualizer {

  /*****************************************************************************
   ** Implementation
   *****************************************************************************/

  QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
  {}

  QNode::~QNode() {
    if (ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
  }

  bool QNode::init() {
    ros::init(init_argc,init_argv,"bwi_exp1_visualizer");
    if ( ! ros::master::check() ) {
      return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    start();
    return true;
  }

  void QNode::on_timeSlider_valueChanged(int value) {
    if (current_time_step_ != value) {
      updateFrameInfo(time);
    }
  }

  void QNode::on_autoPlayBox_toggled(bool checked) {
    autoplay = checked;
  }

  void QNode::run() {
    ros::Rate loop_rate(1.0/time_step_);
    while (ros::ok()) {
      if (autoplay) {
        emit incrementTime();
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
  }

}  // namespace bwi_exp1_visualizer
