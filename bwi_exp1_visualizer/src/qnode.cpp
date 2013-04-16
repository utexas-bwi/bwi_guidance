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

#include <topological_mapper/map_loader.h>

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
    if (!ros::master::check()) {
      return false;
    }
    ros::start(); // explicitly needed since nodehandle going out of scope

    // Get the robot positioner service
    ros::NodeHandle nh;
    robot_positioner_ = nh.serviceClient<bwi_msgs::PositionRobot>("position");
    ROS_INFO_STREAM("Waiting for service: /position");
    robot_positioner_->waitForExistence();
    ROS_INFO_STREAM("Service Acquired: /position");

    // Get parameters to understand image generation
    ros::param::get("~map_file", map_file_);
    ros::param::get("~graph_file", graph_file_);
    ros::param::get("~experiment_file", experiment_file_);
    ros::param::get("~data_directory", data_directory_);
    ros::param::get("~users_file", users_file_);

    /* Initialize the map and associated data now the we know where the map 
       file is */
    mapper_.reset(new topological_mapper::MapLoader(map_file_));
    nav_msgs::MapMetaData info;
    mapper_->getMapInfo(info);
    topological_mapper::readGraphFromFile(graph_file_, info, graph_);

    /* Process experiment data */
    /* Process user data */
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
