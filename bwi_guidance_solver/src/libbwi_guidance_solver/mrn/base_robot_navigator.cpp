#include <algorithm>
#include <boost/foreach.hpp>
#include <boost/range/adaptor/map.hpp>
#include <cstdlib>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv/highgui.h>
#include <tf/transform_datatypes.h>

#include <bwi_guidance_solver/mrn/base_robot_navigator.h>

namespace bwi_guidance_solver {

  namespace mrn {

    BaseRobotNavigator::BaseRobotNavigator(const boost::shared_ptr<ros::NodeHandle>& nh,
                                           const boost::shared_ptr<TaskGenerationModel>& model) {

      nh_ = nh;
      task_generation_model_ = model;

      // This decides whether the MRN has frozen the list of available robots and started controlling them to perform
      // various background tasks, even if the human-guidance task is not being performed.
      multi_robot_navigator_started_ = false;

      // This decides whether an episode inside the MDP is running or not.
      instance_in_progress_ = false;

      // TODO: Parametrize!
      controller_thread_frequency_ = 2.0f;

      // Read the graph from file.
      std::string map_file, graph_file;
      double robot_radius, robot_padding;
      if (!private_nh.getParam("map_file", map_file)) {
        ROS_FATAL_STREAM("RobotPosition: ~map_file parameter required");
        exit(-1);
      }
      if (!private_nh.getParam("graph_file", graph_file)) {
        ROS_FATAL_STREAM("RobotPosition: ~graph_file parameter required");
        exit(-1);
      }

      // Setup map, graph.
      mapper_.reset(new bwi_mapper::MapLoader(map_file));
      mapper_->getMapInfo(map_info_);
      mapper_->getMap(map_);
      bwi_mapper::readGraphFromFile(graph_file, map_info_, graph_);

      human_location_available_ = false;
      human_location_subscriber_ = nh->subscribe("/person/odom", 1, &BaseRobotNavigator::humanLocationHandler, this);

      // Subscribe to the list of available robots
      available_robots_subscriber_ =
        nh->subscribed("/available_robots", 1, &BaseRobotNavigator::availableRobotHandler, this);

      // Start the service server that will eventually start the multi robot navigator controller thread.
      start_server_ = nh->advertiseService("~start", &BaseRobotNavigator::startMultiRobotNavigator, this);
    }

    void BaseRobotNavigator::humanLocationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr human_pose) {
      human_location_ = human_pose->pose.pose;
    }

    void BaseRobotNavigator::availableRobotHandler(const bwi_msgs::AvailableRobotArray::ConstPtr available_robots) {
      if (!multi_robot_navigator_started_) {
        BOOST_FOREACH(const bwi_msgs::AvailableRobot &robot, available_robots->robots) {
          if (std::find(available_robot_list_.begin(), available_robot_list_.end(), robot.name) == available_robot_list_.end()) {
            available_robot_list_.push_back(robot.name);
          }
        }
      }
    }

    void BaseRobotNavigator::startMultiRobotNavigator(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
      if (!multi_robot_navigator_started_) {
        multi_robot_navigator_started_ = true; // This freezes the list of robots.
        // Setup variables/controllers for reach robot.
        int num_robots = available_robot_list_.size();
        for (int i = 0; i < num_robots; ++i) {
          const std::string &robot_name = available_robot_list_[i];

          robot_location_available_.push_back(false);
          robot_location_.push_back(geometry_msgs::Pose()); // We're just resizing the vector here.

          // Subscribe to this robot's location.
          boost::shared_ptr<ros::Subscriber> sub(new ros::Subscriber);
          *sub = nh->subscribe<geometry_msgs::PoseWithCovarianceStamped>("/" + robot_name + "/amcl_pose",
                                                                         1,
                                                                         boost::bind(&BaseRobotNavigator::robotLocationHandler,
                                                                                     this,
                                                                                     _1,
                                                                                     i));
          robot_location_subscriber_.push_back(sub);

          // Add a controller for this robot.
          boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > as;
          as.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("/" + robot_name + "/move_base_interruptable", true));
          robot_controller_.push_back(as);

          // Finally get a new task for this robot and setup the system state for this robot.
          RobotState rs;
          // rs.loc will be setup once the subscriber kicks in. set the loc to -1 for now to indicate the location has
          // not been received.
          rs.loc_u = -1;
          rs.loc_v = -1;
          rs.loc_p = 0.0f;
          rs.help_destination = NONE;

          // TODO: The first task should just make them go to the closest node and sit there.
          getNextTaskForRobot(i, rs);

          system_state.robots.push_back(rs);
        }

        // Now that all robots are initialized, start the controller thread.
        controller_thread_.reset(new boost::thread(&BaseRobotNavigator::runControllerThread, this));
      } else {
        ROS_WARN_NAMED("MultiRobotNavigator", "has already started, cannot start again!");
      }
    }

    void BaseRobotNavigator::robotLocationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr robot_pose,
                                                  int robot_idx) {

      robot_location_available_[robot_idx] = true;
      robot_location_[robot_idx] = robot_pose->pose.pose;
    }


    void BaseRobotNavigator::runControllerThread() {
      ros::Rate r(controller_thread_frequency_);
      while(ros::ok()) {
        // Let's see if we can update the position of all the robots first!
        BOOST_FOREACH(RobotState &rs, extended_state.robots) {
          if (rs.loc_u == -1) {
            // This robot's location has never been initialized. Initialize it using the

          }

        }
        ros

      }
    }

} /* bwi_guidance_solver */
