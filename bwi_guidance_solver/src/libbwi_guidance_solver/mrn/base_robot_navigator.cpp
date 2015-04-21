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
      episode_in_progress_ = false;

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

        robot_location_available_.resize(num_robots, false);
        robot_location_.resize(num_robots); // We're just resizing the vector here.
        robot_location_mutex_.resize(num_robots);
        robot_service_task_start_time_.resize(num_robots);

        for (int i = 0; i < num_robots; ++i) {
          const std::string &robot_name = available_robot_list_[i];

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

          getNextTaskForRobot(i, rs);
          // TODO Send this command

          system_state_.robots.push_back(rs);
        }

        // Now that all robots are initialized, start the controller thread.
        controller_thread_.reset(new boost::thread(&BaseRobotNavigator::runControllerThread, this));
      } else {
        ROS_WARN_NAMED("MultiRobotNavigator", "has already started, cannot start again!");
      }
    }

    void BaseRobotNavigator::robotLocationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr robot_pose,
                                                  int robot_idx) {
      boost::scoped_lock<boost::mutex> lock(robot_location_mutex_[robot_idx]);
      robot_location_[robot_idx] = robot_pose->pose.pose;
      robot_location_available_[robot_idx] = true;
    }


    void BaseRobotNavigator::runControllerThread() {

      // TODO figure out how you want to run this while computing/not-computing given that an instance is in progress.
      while(ros::ok()) {

        bool sleep_via_system_time = true;

        std::vector<bool> send_robot_command(system_state_.robots.size(), false);

        bool all_robot_locations_available_ = true;
        // Let's see if we can update the position of all the robots first!
        for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
          RobotState &rs = system_state_.robots[robot_idx];
          if (rs.loc_u == -1) {
            // This robot's location has never been initialized.
            boost::scoped_lock lock(robot_location_mutex_[robot_idx]);
            if (robot_location_available_[robot_idx]) {
              determineStartLocation(robot_location_[robot_idx], rs.loc_u, rs.loc_v, rs.loc_p);
              roundOffRobotLocation(rs);
            } else {
              all_robot_locations_available_ = false;
            }
          } else { 
            determineRobotTransitionalLocation(robot_location_[robot_idx], rs);
            actionlib::SimpleActionClient robot_state = robot_controller_[robot_idx]->getState(); 
            if (robot_state == actionlib::SimpleClientGoalState::SUCCEEDED ||
              robot_state == actionlib::SimpleClientGoalState::LOST) {
              roundOffRobotLocation(rs);
            } else if (robot_state == actionlib::SimpleClientGoalState::RECALLED ||
                       robot_state == actionlib::SimpleClientGoalState::REJECTED ||
                       robot_state == actionlib::SimpleClientGoalState::PREEMPTED ||
                       robot_state == actionlib::SimpleClientGoalState::ABORTED) {
              // TODO If status is anything other than aborted, then print a warning.
              send_robot_command[robot_idx] = true;
            }
          }
        }

        if (all_robot_locations_available_) {
          bool its_decision_time = false;
          if (episode_in_progress_) {
            if (at_episode_start_) {
              int loc;
              determineStartLocation(human_location_, loc);
              system_state_.loc_node = system_state_.loc_prev = loc;
              at_episode_start_ = false;
              its_decision_time = true;
            } else {
              determineHumanTransitionalLocation(human_location_, system_state_.loc_node, next_loc); 
              if (next_loc != system_state_.loc_node) {
                if (next_loc == goal_node_id_) {
                  // TODO Handle terminal case - Make sure any robot that were assigned is released, and the episode is
                  // marked as completed.
                } else {
                system_state_.loc_prev = system_state_.loc_node;
                system_state_.loc_node = next_loc;
                its_decision_time = true;
              }
            }
          }

          if (its_decision_time) {
            while (true) {
              Action action = getBestAction();
              if (action.type != WAIT) {
                // Perform the deterministic transition as per the model 
                ExtendedState next_state;
                bool unused_terminal; /* The resulting state can never be terminal via a non WAIT action */
                float unused_reward_value;
                int unused_depth_count;

                // Note that the RNG won't be used as it is a deterministic action.
                model->takeAction(system_state_, action, unused_reward, next_state, unused_terminal, unused_depth_count, rng);
                for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
                  if (system_state_.robots[robot_idx].help_destination != next_state.robots[robot_idx].help_destination) {
                    send_robot_command[robot_idx] = true;
                  }
                }
                system_state_ = next_state;
              } else {
                // Let's switch to non-deterministic transition logic.
                break;
              }
            }
          }

          for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
            RobotState &rs = system_state_.robots[robot_idx];
            if (send_robot_command[robot_idx]) {
              int destination = (rs.help_destination != NONE) ? rs.help_destination : rs.tau_d;
              // TODO construct going to this destination. Select orientation intelligently. 
              {
                

              }
            } else {
              if (rs.help_destination != NONE) {
                if (isRobotExactlyAt(rs, rs.help_destination)) {
                  // TODO See if the robot needs to be rotated to be in a good orientation w.r.t to the person. 
                  // Otherwise, do nothing.
                }
              } else {
                if (isRobotExactlyAt(rs, rs.tau_d)) {

                  // The robot is at the service task location performing the task. Figure out how much time
                  // the task has been performed for.
                  if (rs.tau_t == 0.0f) {
                    rs.tau_t = 1.0f / controller_thread_frequency_;
                    // The second term tries to average for discretization errors. TODO think about this some more when
                    // not sleepy.
                    robot_service_task_start_time_[robot_idx] = boost::posix_time::microsec_clock::local_time() -
                      boost::posix_time::milliseconds(0.5 * 1.0f / controller_thread_frequency_); 
                  } else {
                    boost::posix_time::time_duration diff = 
                      boost::posix_time::microsec_clock::local_time() - robot_service_task_start_time_[robot_idx];
                    rs.tau_t = diff.total_milliseconds();
                  }

                  if (rs.tau_t > rs.tau_T) {
                    // This service task is now complete. Get a new task.
                    getNextTaskForRobot(robot_idx, rs); 
                    // TODO Send this command.
                  }
                }
              }

            }
          }
        } else {
          ROS_WARN_THROTTLE_NAMED(1.0f, "MultiRobotNavigator", " still waiting for robot locations. This shouldn't take too long...");
        }

        if (sleep_via_system_time) {
          boost::this_thread::sleep(boost::posix_time::milliseconds(1.0f/controller_thread_frequency_));
        } else {
          compute(1.0f/controller_thread_frequency_);
        }
      }

    }

    void BaseRobotNavigator::determineStartLocation(const geometry_msgs::Pose loc, int &u, int &v, float p) {
      // Initialize it by finding the node it is closest to, and then the edge from that node to determine v and p.

    }

} /* bwi_guidance_solver */
