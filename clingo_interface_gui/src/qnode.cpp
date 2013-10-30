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

#include <boost/thread/thread.hpp>

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

    std::string map_file, door_file, location_file;
    bool use_sim_time;
    ros::param::get("~map_file", map_file);
    ros::param::get("~door_file", door_file);
    ros::param::get("~location_file", location_file);
    ros::param::param<std::string>("~global_frame", global_frame_, "map");
    ros::param::param("~sim_auto_door", sim_auto_door_, true);
    ros::param::param("/use_sim_time", use_sim_time, false);
    if (sim_auto_door_) {
      ROS_INFO("ClingoInterface: Running in simulation!");
    } else {
      ROS_INFO("ClingoInterface: Running in the real world!");
    }
    if (sim_auto_door_ && !use_sim_time) {
      ROS_ERROR_STREAM("Automated door interaction enabled, but no simulator present!");
      sim_auto_door_ = false;
    }

    close_door_idx_ = -1;
    prev_door_idx_ = -1;
    handler_.reset(new clingo_interface::DoorHandler(map_file, door_file, location_file));
    if (sim_auto_door_) {
      gh_.reset(new clingo_interface::GazeboHandler);
    }
    ce_.reset(new clingo_interface::CostEstimator);

    tf_.reset(new tf::TransformListener);
    odom_subscriber_.reset(new message_filters::Subscriber<nav_msgs::Odometry>);
    odom_subscriber_->subscribe(*nh_, "odom", 5);
    tf_filter_.reset(new tf::MessageFilter<nav_msgs::Odometry>(
        *odom_subscriber_, *tf_, global_frame_, 5));
    tf_filter_->registerCallback(boost::bind(&QNode::odometryHandler, this, _1));

    as_.reset(new actionlib::SimpleActionServer<
        clingo_interface_gui::ClingoInterfaceAction>(*nh_, "clingo_interface_gui", 
            boost::bind(&QNode::clingoInterfaceHandler, this, _1), false));
    as_->start();

    robot_controller_.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true));
    robot_controller_->waitForServer();

    ROS_INFO_STREAM("ClingoInterface: move_base server is now AVAILABLE");

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
          location_box_enabled_ = false;
          display_text_ = req->command.op + " " + door_name;
          Q_EMIT updateFrameInfo();

          geometry_msgs::PoseStamped pose;
          pose.header.stamp = ros::Time::now();
          pose.header.frame_id = "/map"; //TODO
          pose.pose.position.x = approach_pt.x;
          pose.pose.position.y = approach_pt.y;
          tf::quaternionTFToMsg(tf::createQuaternionFromYaw(approach_yaw), 
              pose.pose.orientation); 
          float start_time = ros::Time::now().toSec();
          bool success = executeRobotGoal(pose);
          float cost = ros::Time::now().toSec() - start_time;
          resp.success = success;

          // Publish the observable fluents
          senseDoorProximity(resp.observable_fluents, door_idx);
          if (success) {
            if (prev_door_idx_ != -1 &&
                prev_door_idx_ != door_idx) {
              size_t location_idx = 
                handler_->getLocationIdx(topological_mapper::Point2f(robot_x_, robot_y_));
              std::string at_str = handler_->getLocationString(location_idx);
              ce_->addSample(at_str, prev_door_idx_, door_idx, cost); 
            }
            prev_door_idx_ = door_idx;
          } else {
            prev_door_idx_ = -1;
          }

          // Close a door is opened previously
          if (success) {
            if (sim_auto_door_ && close_door_idx_ != -1) {
              gh_->closeDoor(close_door_idx_);
              close_door_idx_ = -1;
            }
          }

          // We can't sense the door once we go through as we won't be facing
          // it. We should only sense a door when we approach it.
          if (req->command.op == "approach") {
            clingo_interface_gui::ClingoFluent door_open;
            door_open.args.push_back(door_name);
            if (handler_->isDoorOpen(door_idx)) {
              door_open.op = "open";
              resp.observable_fluents.push_back(door_open);
            } else {
              door_open.op = "n_open";
              resp.observable_fluents.push_back(door_open);
            }
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
        location_box_enabled_ = false;
        display_text_ = "Can you please open " + door_name;
        Q_EMIT updateFrameInfo();

        /* Wait for the door to be opened */
        ros::Rate r(1);
        int count = 0;
        bool door_open = false; 
        while (!door_open && count < 30) {
          if (sim_auto_door_ && count == 1) {
            gh_->openDoor(door_idx);
            close_door_idx_ = door_idx;
          }
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
        clingo_interface_gui::ClingoFluent door_open_fluent;
        door_open_fluent.args.push_back(door_name);
        if (!door_open) {
          display_text_ = "Oh no! The request timed out. Let me think...";
          Q_EMIT updateFrameInfo();
          door_open_fluent.op = "n_open";
          resp.success = false;
          resp.status = "User unable to open door";
        } else {
          display_text_ = "Thank you for opening the door!";
          Q_EMIT updateFrameInfo();
          door_open_fluent.op = "open";
          resp.success = true;
        }
        resp.observable_fluents.push_back(door_open_fluent);
      }
    } else if (req->command.op == "sense") {
      std::string sense_fluent_op = req->sense_fluent.op;
      if (sense_fluent_op == "open") {
        std::string door_name = req->sense_fluent.args[0];
        size_t door_idx = handler_->getDoorIdx(door_name);
        clingo_interface_gui::ClingoFluent door_open;
        door_open.args.push_back(door_name);
        if (handler_->isDoorOpen(door_idx)) {
          door_open.op = "open";
          resp.observable_fluents.push_back(door_open);
        } else {
          door_open.op = "n_open";
          resp.observable_fluents.push_back(door_open);
        }
        resp.success = true;
      } else if (sense_fluent_op == "beside" || sense_fluent_op == "facing") {
        std::string door_name = req->sense_fluent.args[0];
        size_t door_idx = handler_->getDoorIdx(door_name);
        topological_mapper::Point2f robot_loc(robot_x_, robot_y_);
        bool op_door = (sense_fluent_op == "beside") ? 
          handler_->isRobotBesideDoor(robot_loc, robot_yaw_, 1.2, door_idx) :
          handler_->isRobotFacingDoor(robot_loc, robot_yaw_, 1.2, door_idx);
        if (!op_door) {
          clingo_interface_gui::ClingoFluent n_op;
          n_op.op = "n_" + sense_fluent_op;
          n_op.args.push_back(door_name);
          resp.observable_fluents.push_back(n_op);
        } else {
          clingo_interface_gui::ClingoFluent op;
          op.op = sense_fluent_op;
          op.args.push_back(door_name);
          resp.observable_fluents.push_back(op);
        }
        resp.success = true;
      } else if (sense_fluent_op == "ploc") {
        person_name_ = req->sense_fluent.args[0];
        /* Update the display */
        button1_enabled_ = false;
        button2_enabled_ = false;
        location_box_enabled_ = true;
        display_text_ = "Can you tell me where " + person_name_ + " is?";
        location_received_ = false;
        Q_EMIT updateFrameInfo();

        /* Wait for the location to be received */
        ros::Rate r(10);
        int count = 0;
        bool door_open = false; 
        while (!location_received_ && count < 300) {
          if (as_->isPreemptRequested() || !ros::ok()) { // TODO What about goal not being active or new goal?
            ROS_INFO("Preempting action");
            as_->setPreempted();
            return;
          }
          count++;
          r.sleep();
        }

        if (location_received_) {
          display_text_ = "Thank you! I will try and find " + person_name_ + " in " + person_location_;
          clingo_interface_gui::ClingoFluent person_loc_fluent;
          person_loc_fluent.op = "inside";
          person_loc_fluent.args.push_back(person_name_);
          person_loc_fluent.args.push_back(person_location_);
          resp.observable_fluents.push_back(person_loc_fluent);
        } else {
          display_text_ = "Oh no! The request timed out. Let me think...";
        }
        location_box_enabled_ = false; 
        Q_EMIT updateFrameInfo();

      }
    } else if (req->command.op == "evaluate") {
      resp.plan_cost = 0;
      resp.success = true;
      topological_mapper::Point2f prev_pt(robot_x_, robot_y_), approach_pt;
      float prev_yaw = robot_yaw_, approach_yaw = 0;
      for (size_t i = 0; i < req->evaluate_fluents.size(); ++i) {
        if (req->evaluate_fluents[i].op == "approach") {
          std::string door_name = req->evaluate_fluents[i].args[0];
          std::cout << "door_name: " << door_name << std::endl;
          size_t door_idx = handler_->getDoorIdx(door_name);
          bool door_approachable = handler_->getApproachPoint(door_idx, 
              prev_pt, approach_pt, approach_yaw);
          if (!door_approachable) {
            resp.plan_cost = std::numeric_limits<float>::max();
            resp.success = false;
            resp.status = "unable to evaluate one approach leg of plan.";
            break;
          }
          resp.plan_cost += handler_->getPathCost(prev_pt, prev_yaw, approach_pt, approach_yaw);
        } else if (req->evaluate_fluents[i].op == "gothrough") {
          std::string door_name = req->evaluate_fluents[i].args[0];
          size_t door_idx = handler_->getDoorIdx(door_name);
          bool door_approachable = handler_->getThroughDoorPoint(door_idx, 
              prev_pt, approach_pt, approach_yaw);
          if (!door_approachable) {
            resp.plan_cost = std::numeric_limits<float>::max();
            resp.success = false;
            resp.status = "unable to evaluate one gothrough leg of plan.";
            std::cout << "from " << prev_pt << " to " << approach_pt << std::endl;
            break;
          }
          resp.plan_cost += 
            topological_mapper::getMagnitude(approach_pt - prev_pt); // don't worry about closed doors for now
        } else {
          continue;
        }

        prev_pt = approach_pt;
        prev_yaw = approach_yaw;
      }
    } else if (req->command.op == "goto") {
      /* Update the display */
      button1_enabled_ = false;
      button2_enabled_ = false;
      location_box_enabled_ = false;
      display_text_ = "Hello " + req->command.args[0] + "!!";
      Q_EMIT updateFrameInfo();
      clingo_interface_gui::ClingoFluent visited;
      visited.op = "visited";
      visited.args.push_back(req->command.args[0]);
      resp.observable_fluents.push_back(visited);
      resp.success = true;
    } else if (req->command.op == "noop") {
      resp.success = true;
      senseDoorProximity(resp.observable_fluents);
    } else if (req->command.op == "finish") {
      ROS_INFO("RECEIVED FINISH");
      resp.success = true;
      prev_door_idx_ = -1; // Robot may be teleported
      ce_->finalizeEpisode();
    }

    // Get location
    size_t location_idx = 
      handler_->getLocationIdx(topological_mapper::Point2f(robot_x_, robot_y_));
    if (location_idx == (size_t) -1) {
      ROS_ERROR("Unable to compute position");
    } else {
      std::string at_str = handler_->getLocationString(location_idx);
      clingo_interface_gui::ClingoFluent at;
      at.op = "at";
      at.args.push_back(at_str);
      resp.observable_fluents.push_back(at);
    }

    /* boost::this_thread::sleep(boost::posix_time::seconds(1)); */

    if (resp.success) {
      as_->setSucceeded(resp, resp.status);
    } else {
      as_->setAborted(resp, resp.status);
    }
  }

  void QNode::odometryHandler(
      const nav_msgs::Odometry::ConstPtr& odom) {
    geometry_msgs::PoseStamped pose_in, pose_out;
    pose_in.header = odom->header;
    pose_in.pose = odom->pose.pose;
    tf_->transformPose(global_frame_, pose_in, pose_out);
    std::cout << pose_out.header.frame_id << std::endl;
    robot_x_ = pose_out.pose.position.x;
    robot_y_ = pose_out.pose.position.y;
    robot_yaw_ = tf::getYaw(pose_out.pose.orientation);
    if (sim_auto_door_) {
      gh_->closeAllDoorsFarAwayFromPoint(pose_out.pose);
    }
  }

  bool QNode::newLocationReceived(const std::string& loc) {
    std::cout << "testing location: " << loc << std::endl;
    if (handler_->getLocationIdx(loc) != (size_t) -1) {
      location_received_ = true;
      person_location_ = loc;
      return true;
    }
    return false;
  }

  void QNode::senseDoorProximity( 
      std::vector<clingo_interface_gui::ClingoFluent>& fluents,
      int door_idx) {
    size_t num_doors = handler_->getNumDoors();
    topological_mapper::Point2f robot_loc(robot_x_, robot_y_);
    for (size_t door = 0; door < num_doors; ++door) {
      if (door_idx != -1 && door != door_idx) {
        continue;
      }
      bool facing_door = handler_->isRobotFacingDoor(
          robot_loc, robot_yaw_, 1.2, door);
      bool beside_door = handler_->isRobotBesideDoor(
          robot_loc, robot_yaw_, 1.2, door);
      if (!facing_door) {
        clingo_interface_gui::ClingoFluent n_facing;
        n_facing.op = "n_facing";
        n_facing.args.push_back(handler_->getDoorString(door));
        fluents.push_back(n_facing);
      } else {
        clingo_interface_gui::ClingoFluent facing;
        facing.op = "facing";
        facing.args.push_back(handler_->getDoorString(door));
        fluents.push_back(facing);
      }
      if (!beside_door) {
        clingo_interface_gui::ClingoFluent n_beside;
        n_beside.op = "n_beside";
        n_beside.args.push_back(handler_->getDoorString(door));
        fluents.push_back(n_beside);
      } else {
        clingo_interface_gui::ClingoFluent beside;
        beside.op = "beside";
        beside.args.push_back(handler_->getDoorString(door));
        fluents.push_back(beside);
      }
    }
  }

  bool QNode::executeRobotGoal(const geometry_msgs::PoseStamped& pose) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = pose;
    robot_controller_->sendGoal(goal);
    robot_controller_->waitForResult();
    actionlib::SimpleClientGoalState state = robot_controller_->getState();
    return state == actionlib::SimpleClientGoalState::SUCCEEDED;
  }

  void QNode::run() {
    ros::spin();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
  }

}  // namespace clingo_interface_gui
