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
#include <clingo_interface_gui/qnode.hpp>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace clingo_interface_gui {

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
      nh_.reset();
      ros::waitForShutdown();
    }
    wait();
  }

  bool QNode::init() {
    ros::init(init_argc,init_argv, "clingo_interface_gui");
    nh_.reset(new ros::NodeHandle);

    ros::param::get("~map_file", map_file_);
    ros::param::get("~door_file", door_file_);
    ros::param::get("~location_file", location_file_);

    // Add your ros communications here.
    tf_.reset(new tf::TransformListener(ros::Duration(10)));
    mapper_.reset(new topological_mapper::MapLoader(map_file_));
    handler_.reset(
        new clingo_interface::DoorHandler(mapper_, door_file_, 
            location_file_, *tf_));

    odom_subscriber_ = nh_->subscribe("odom", 1, &QNode::odometryHandler, this);
    as_.reset(new actionlib::SimpleActionServer<
        clingo_interface_gui::ClingoInterfaceAction>(*nh_, "clingo_interface_gui", 
            boost::bind(&QNode::clingoInterfaceHandler, this, _1), false));
    as_->start();

    robot_controller_ = nh_->advertise<geometry_msgs::PoseStamped>(
        "move_base_simple/goal", 1);

    start();
    return true;
  }

  void QNode::clingoInterfaceHandler(
      const clingo_interface_gui::ClingoInterfaceGoalConstPtr &req) {

    clingo_interface_gui::ClingoInterfaceResult resp;

    if (req->command.op == "approach" || req->command.op == "gothrough") {
      std::string door_name = req->command.args[0];
      size_t door_idx = handler_->getDoorIdx(door_name);
      if (door_idx == (size_t)-1) {
        // Interface failure
        resp.success = false;
        resp.status = "Could not resolve argument: " + door_name;
      } else {
        topological_mapper::Point2f approach_pt;
        float approach_yaw = 0;
        bool door_approachable = false;
        if (req->command.op == "approach") {
          door_approachable = handler_->getApproachPoint(door_idx, 
              topological_mapper::Point2f(robot_x_, robot_y_), approach_pt,
              approach_yaw);
        } else {
          door_approachable = handler_->getThroughDoorPoint(door_idx, 
              topological_mapper::Point2f(robot_x_, robot_y_), approach_pt,
              approach_yaw);
        }
        if (door_approachable) {

          /* Update the display */
          button1_enabled_ = false;
          button2_enabled_ = false;
          display_text_ = req->command.op + " " + door_name;
          Q_EMIT updateFrameInfo();

          geometry_msgs::PoseStamped pose;
          pose.header.stamp = ros::Time::now();
          pose.header.frame_id = "/map"; //TODO
          pose.pose.position.x = approach_pt.x;
          pose.pose.position.y = approach_pt.y;
          tf::quaternionTFToMsg(tf::createQuaternionFromYaw(approach_yaw), 
              pose.pose.orientation); 
          robot_controller_.publish(pose); // TODO Use the action lib API for move base and provide feedback here
          resp.success = true;

          // Publish the observable fluents
          // TODO Besides should be published if approach goal succeeds
          if (req->command.op == "approach") {
            clingo_interface_gui::ClingoFluent beside;
            beside.op = "beside";
            beside.args.push_back(door_name);
            resp.observable_fluents.push_back(beside);
          }
          clingo_interface_gui::ClingoFluent door_open;
          door_open.op = "open";
          door_open.args.push_back(door_name);
          if (handler_->isDoorOpen(door_idx)) {
            resp.observable_fluents.push_back(door_open);
          }

        } else {
          // Planning failure
          resp.success = false;
          resp.status = "Cannot approach " + door_name + " from here.";
        }

      }
    } else if (req->command.op == "callforopen") {
      std::string door_name = req->command.args[0];
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

        /* Wait for the door to be opened */
        ros::Rate r(10);
        int count = 0;
        bool door_open = false; 
        while (!door_open && count < 300) {
          if (as_->isPreemptRequested() || !ros::ok()) { // TODO What about goal not being active or new goal?
            ROS_INFO("Preempting action");
            as_->setPreempted();
            return;
          }
          door_open = handler_->isDoorOpen(door_idx);
          count++;
          r.sleep();
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
    } else if (req->command.op == "goto") {
      /* Update the display */
      button1_enabled_ = false;
      button2_enabled_ = false;
      display_text_ = "Hello " + req->command.args[0] + "!!";
      Q_EMIT updateFrameInfo();
      resp.success = true;
    }

    // Get location
    size_t location_idx = 
      handler_->getLocationIdx(topological_mapper::Point2f(robot_x_, robot_y_));
    if (location_idx == (size_t) -1) {
      ROS_ERROR("Unable to compute position");
    } else {
      std::string location_str = handler_->getLocationString(location_idx);
      clingo_interface_gui::ClingoFluent location;
      location.op = "location";
      location.args.push_back(location_str);
      resp.observable_fluents.push_back(location);
    }

    if (resp.success) {
      as_->setSucceeded(resp, resp.status);
    } else {
      as_->setAborted(resp, resp.status);
    }
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

}  // namespace clingo_interface_gui
