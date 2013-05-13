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
#include <clingo_interface/qnode.hpp>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace clingo_interface {

  /*****************************************************************************
   ** Implementation
   *****************************************************************************/

  QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
  {
    generated_image_ = cv::Mat::zeros(300,300,CV_8UC3);
  }

  QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
  }

  bool QNode::init() {
    ros::init(init_argc,init_argv, "clingo_interface");
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    ros::param::get("~map_file", map_file_);
    ros::param::get("~door_file", door_file_);
    ros::param::get("~location_file", location_file_);

    // Add your ros communications here.
    tf_.reset(new tf::TransformListener(ros::Duration(10)));
    mapper_.reset(new topological_mapper::MapLoader(map_file_));
    handler_.reset(
        new clingo_helpers::DoorHandler(mapper_, door_file_, 
            location_file_, *tf_));

    service_ = n.advertiseService("clingo_interface", 
          &QNode::clingoInterfaceHandler, this);
    odom_subscriber_ = n.subscribe("odom", 1, &QNode::odometryHandler, this);

    robot_controller_ = n.advertise<geometry_msgs::PoseStamped>(
        "move_base_simple/goal", 1);

    start();
    return true;
  }

  bool QNode::clingoInterfaceHandler(
      clingo_helpers::ClingoInterface::Request &req,
      clingo_helpers::ClingoInterface::Response &resp) {

    if (req.command.op == "approach") {
      std::string door_name = req.command.args[0];
      size_t door_idx = handler_->getDoorIdx(door_name);
      if (door_idx == (size_t)-1) {
        resp.success = false;
        resp.status = "Could not resolve argument: " + door_name;
      } else {
        topological_mapper::Point2f approach_pt;
        float approach_yaw = 0;
        bool door_approachable = handler_->getApproachPoint(door_idx, 
            topological_mapper::Point2f(robot_x_, robot_y_), approach_pt,
            approach_yaw);
        if (door_approachable) {
          geometry_msgs::PoseStamped pose;
          pose.header.stamp = ros::Time::now();
          pose.header.frame_id = "/map"; //TODO
          pose.pose.position.x = approach_pt.x;
          pose.pose.position.y = approach_pt.y;
          tf::quaternionTFToMsg(tf::createQuaternionFromYaw(approach_yaw), 
              pose.pose.orientation); //TODO
          robot_controller_.publish(pose);
          resp.success = true;
        } else {
          resp.success = false;
          resp.status = "Cannot approach " + door_name + " from here.";
        }
      }
    } else if (req.command.op == "gothrough") {
      std::string door_name = req.command.args[0];
      size_t door_idx = handler_->getDoorIdx(door_name);
      if (door_idx == (size_t)-1) {
        resp.success = false;
        resp.status = "Could not resolve argument: " + door_name;
      } else {
        topological_mapper::Point2f approach_pt;
        float approach_yaw = 0;
        bool door_approachable = handler_->getThroughDoorPoint(door_idx, 
            topological_mapper::Point2f(robot_x_, robot_y_), approach_pt,
            approach_yaw);
        if (door_approachable) {
          geometry_msgs::PoseStamped pose;
          pose.header.stamp = ros::Time::now();
          pose.header.frame_id = "/map"; //TODO
          pose.pose.position.x = approach_pt.x;
          pose.pose.position.y = approach_pt.y;
          tf::quaternionTFToMsg(tf::createQuaternionFromYaw(approach_yaw), 
              pose.pose.orientation); //TODO
          robot_controller_.publish(pose);
          resp.success = true;
        } else {
          resp.success = false;
          resp.status = "Cannot approach " + door_name + " from here.";
        }
      }
    } else if (req.command.op == "callforopen") {
      std::string door_name = req.command.args[0];
      size_t door_idx = handler_->getDoorIdx(door_name);
      if (door_idx == (size_t)-1) {
        resp.success = false;
        resp.status = "Could not resolve argument: " + door_name;
      } else {

        /* Update the display */
        button1_enabled_ = false;
        button2_enabled_ = false;
        display_text_ = "Can you please open " + door_name;
        Q_EMIT updateFrameInfo();

        /* Wait for door to be opent */
        int count = 0;
        bool door_open = 
            handler_->isDoorOpen(handler_->getLocationIdx(door_name));
        while (!door_open && count < 300) {
          std::cout << door_open << std::endl;
          boost::this_thread::sleep(boost::posix_time::milliseconds(100));
          door_open = handler_->isDoorOpen(handler_->getLocationIdx(door_name));
          std::cout << "Door not open" << std::endl;
          //regenerateImage();
          count++;
        }

        /* Update the display and send back appropriate fluents */
        if (!door_open) {
          display_text_ = "Oh no! The request timed out. Let me think...";
          Q_EMIT updateFrameInfo();
          resp.success = false;
          resp.status = "User unable to open door";
        } else {
          display_text_ = "Thank you for opening the door!";
          Q_EMIT updateFrameInfo();
          resp.success = true;
        }
      }
    } else if (req.command.op == "goto") {
      /* Update the display */
      button1_enabled_ = false;
      button2_enabled_ = false;
      display_text_ = "Hello " + req.command.args[0] + "!!";
      Q_EMIT updateFrameInfo();
    }
    return true;
  }

  void QNode::odometryHandler(
      const nav_msgs::Odometry::ConstPtr& odom) {
    robot_x_ = odom->pose.pose.position.x;
    robot_y_ = odom->pose.pose.position.y;
    robot_yaw_ = tf::getYaw(odom->pose.pose.orientation);
  }

  void QNode::run() {
    ros::spin();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
  }

}  // namespace clingo_interface
