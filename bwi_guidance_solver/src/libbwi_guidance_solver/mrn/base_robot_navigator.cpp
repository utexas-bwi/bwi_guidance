#include <algorithm>
#include <boost/foreach.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <cstdlib>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv/highgui.h>
#include <tf/transform_datatypes.h>

#include <bwi_guidance_msgs/UpdateGuidanceGui.h>
#include <bwi_guidance_solver/mrn/abstract_mapping.h>
#include <bwi_guidance_solver/mrn/base_robot_navigator.h>
#include <bwi_guidance_solver/mrn/single_robot_solver.h>
#include <bwi_rl/planning/ModelUpdaterSingle.h>

#include <opencv/highgui.h>

namespace bwi_guidance_solver {

  namespace mrn {

    BaseRobotNavigator::BaseRobotNavigator(const boost::shared_ptr<ros::NodeHandle>& nh,
                                           const std::vector<std::string>& available_robot_list,
                                           const boost::shared_ptr<TaskGenerationModel>& model) {

      nh_ = nh;

      available_robot_list_ = available_robot_list;

      // This decides whether an episode inside the MDP is running or not.
      episode_in_progress_ = false;
      episode_completed_ = false;
      terminate_episode_ = false;
      at_episode_start_ = false;
      pause_robot_ = NONE;

      // TODO: Parametrize!
      controller_thread_frequency_ = 2.0f;
      avg_human_speed_ = 0.75f;
      avg_robot_speed_ = 0.4f;

      // Read the graph from file.
      std::string map_file, graph_file;
      double robot_radius, robot_padding;
      ros::NodeHandle private_nh("~");
      if (!private_nh.getParam("map_file", map_file)) {
        ROS_FATAL_STREAM("RobotPosition: ~map_file parameter required");
        exit(-1);
      }
      if (!private_nh.getParam("graph_file", graph_file)) {
        ROS_FATAL_STREAM("RobotPosition: ~graph_file parameter required");
        exit(-1);
      }

      // Setup map, graph and random number generator.
      mapper_.reset(new bwi_mapper::MapLoader(map_file));
      mapper_->getMapInfo(map_info_);
      mapper_->getMap(map_);
      mapper_->drawMap(base_image_);
      bwi_mapper::readGraphFromFile(graph_file, map_info_, graph_);
      master_rng_.reset(new RNG(0));

      // TODO just initialize the task generation model from scratch here.
      task_generation_model_ = model;
      motion_model_.reset(new MotionModel(graph_, avg_robot_speed_, avg_human_speed_));
      human_decision_model_.reset(new HumanDecisionModel(graph_));

      human_location_available_ = false;
      human_location_subscriber_ = nh_->subscribe("/person/pose", 1, &BaseRobotNavigator::humanLocationHandler, this);

      cvStartWindowThread();
      cv::namedWindow("out");

    }

    BaseRobotNavigator::~BaseRobotNavigator() {
      // TODO join the controller thread here if initialized?
    }

    void BaseRobotNavigator::execute(const bwi_guidance_msgs::MultiRobotNavigationGoalConstPtr &goal) {
      /* restricted mutex scope */ {
        ROS_INFO_NAMED("BaseRobotNavigator", "Execute called!");
        boost::mutex::scoped_lock episode_modification_lock(episode_modification_mutex_);
        episode_completed_ = false;
        terminate_episode_ = false;
        episode_in_progress_ = true;
        at_episode_start_ = true;
        goal_node_id_ = goal->goal_node_id;

        model_.reset(new RestrictedModel(graph_,
                                         map_,
                                         goal_node_id_,
                                         motion_model_,
                                         human_decision_model_,
                                         task_generation_model_,
                                         RestrictedModel::Params()));

        boost::shared_ptr<ModelUpdaterSingle<ExtendedState, Action> >
          mcts_model_updator(new ModelUpdaterSingle<ExtendedState, Action>(model_));
        boost::shared_ptr<StateMapping<ExtendedState> > mcts_state_mapping;
        mcts_state_mapping.reset(new AbstractMapping);

        boost::shared_ptr<SingleRobotSolver> default_policy;
        default_policy.reset(new SingleRobotSolver);

        // TODO fill this from the current solver values.
        Json::Value empty_json; // Force the single robot solver to use default parameters.
        std::vector<int> empty_robot_home_base(available_robot_list_.size(), 0);
        std::string base_directory = "/tmp";
        Domain::Params domain_params;
        default_policy->initialize(domain_params, empty_json, map_, graph_, empty_robot_home_base, base_directory);
        default_policy->reset(0, goal_node_id_);
        MultiThreadedMCTS<ExtendedState, ExtendedStateHash, Action>::Params mcts_params;

        mcts_.reset(new MultiThreadedMCTS<ExtendedState, ExtendedStateHash, Action>(default_policy,
                                                                                    mcts_model_updator,
                                                                                    mcts_state_mapping,
                                                                                    master_rng_,
                                                                                    mcts_params));

        ROS_INFO_NAMED("BaseRobotNavigator", "Execute ended!");
      }

      while (ros::ok()) {
        /* restricted mutex scope */ {
          boost::mutex::scoped_lock episode_modification_lock(episode_modification_mutex_);
          if (as_->isPreemptRequested()) {
            terminate_episode_ = true;
            as_->setAborted();
            break;
          }
          if (episode_completed_) {
            as_->setSucceeded();
            break;
          }
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
      }

    }

    void BaseRobotNavigator::humanLocationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &human_pose) {
      human_location_ = human_pose->pose.pose;
    }

    void BaseRobotNavigator::start() {

      // Setup variables/controllers for reach robot.
      int num_robots = available_robot_list_.size();

      robot_location_available_.resize(num_robots, false);
      robot_location_.resize(num_robots); // We're just resizing the vector here.
      robot_location_mutex_.resize(num_robots);
      for (int i = 0; i < num_robots; ++i) {
        robot_location_mutex_[i].reset(new boost::mutex);
      }

      robot_service_task_start_time_.resize(num_robots);
      robot_command_status_.resize(num_robots, INITIALIZED);

      for (int i = 0; i < num_robots; ++i) {
        const std::string &robot_name = available_robot_list_[i];

        // Add a controller for this robot.
        ROS_INFO_STREAM_NAMED("BaseRobotNavigator", "Waiting for action server for robot: " << robot_name);
        boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > ac;
        ac.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("/" + robot_name + "/move_base_interruptable", true));
        ac->waitForServer();
        robot_controller_.push_back(ac);

        boost::shared_ptr<ros::Subscriber> loc_sub(new ros::Subscriber);
        *loc_sub = nh_->subscribe<geometry_msgs::PoseWithCovarianceStamped>("/" + robot_name + "/amcl_pose", 1,
                                                                            boost::bind(&BaseRobotNavigator::robotLocationHandler,
                                                                                        this, _1, i));
        robot_location_subscriber_.push_back(loc_sub);

        boost::shared_ptr<ros::ServiceClient> gui_client(new ros::ServiceClient);
        *gui_client = nh_->serviceClient<bwi_guidance_msgs::UpdateGuidanceGui>("/" + robot_name + "/update_gui");
        robot_gui_controller_.push_back(gui_client);

        // Finally get a new task for this robot and setup the system state for this robot.
        RobotState rs;
        // rs.loc will be setup once the subscriber kicks in. set the loc to -1 for now to indicate the location has
        // not been received.
        rs.loc_u = -1;
        rs.loc_v = -1;
        rs.loc_p = 0.0f;

        rs.tau_d = -1; // Get Starting task.
        getNextTaskForRobot(i, rs);

        rs.help_destination = NONE;

        system_state_.robots.push_back(rs);
      }

      // Now that all robots are initialized, start the controller thread.
      controller_thread_.reset(new boost::thread(&BaseRobotNavigator::runControllerThread, this));

      as_.reset(new actionlib::SimpleActionServer<bwi_guidance_msgs::MultiRobotNavigationAction>(*nh_,
                                                                                                 "/guidance",
                                                                                                 boost::bind(&BaseRobotNavigator::execute, this, _1),
                                                                                                 false));

      as_->start();
      ros::spin();
    }

    void BaseRobotNavigator::robotLocationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &robot_pose,
                                                  int robot_idx) {
      boost::mutex::scoped_lock lock(*(robot_location_mutex_[robot_idx]));
      robot_location_[robot_idx] = robot_pose->pose.pose;
      robot_location_available_[robot_idx] = true;
    }


    void BaseRobotNavigator::runControllerThread() {

      ROS_INFO_NAMED("BaseRobotNavigator", "Starting up...");

      // TODO figure out how you want to run this while computing/not-computing given that an instance is in progress.
      while(ros::ok()) {

        /* restricted_mutex_scope */ {
          boost::mutex::scoped_lock episode_modification_lock(episode_modification_mutex_);

          bool all_robot_locations_available = true;
          // Let's see if we can update the position of all the robots first!
          for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
            RobotState &rs = system_state_.robots[robot_idx];
            if (rs.loc_u == -1) {
              // This robot's location has never been initialized.
              boost::mutex::scoped_lock lock(*(robot_location_mutex_[robot_idx]));
              if (robot_location_available_[robot_idx]) {
                determineStartLocation(robot_location_[robot_idx], rs.loc_u, rs.loc_v, rs.loc_p);
              } else {
                all_robot_locations_available = false;
              }
            } else if (robot_command_status_[robot_idx] != INITIALIZED) {
              determineRobotTransitionalLocation(robot_location_[robot_idx], rs);
              actionlib::SimpleClientGoalState robot_state = robot_controller_[robot_idx]->getState();
              if (robot_state == actionlib::SimpleClientGoalState::SUCCEEDED ||
                  robot_state == actionlib::SimpleClientGoalState::LOST) {
                roundOffRobotLocation(rs);
                if (robot_command_status_[robot_idx] == GOING_TO_HELP_DESTINATION_LOCATION) {
                  robot_command_status_[robot_idx] = AT_HELP_DESTINATION_LOCATION;
                } else if (robot_command_status_[robot_idx] == GOING_TO_SERVICE_TASK_LOCATION) {
                  robot_command_status_[robot_idx] = AT_SERVICE_TASK_LOCATION;
                  if (!episode_in_progress_) {
                    bwi_guidance_msgs::UpdateGuidanceGui srv;
                    srv.request.type = bwi_guidance_msgs::UpdateGuidanceGuiRequest::ENABLE_EPISODE_START;
                    robot_gui_controller_[robot_idx]->call(srv);
                  }
                }
                // else {
                //   ROS_FATAL_STREAM("BAD STATE TRANSITION 1 - robot successfully completed navigation action in command status" << robot_command_status_[robot_idx]);
                //   throw std::runtime_error("");
                // }
              } else if (robot_state == actionlib::SimpleClientGoalState::RECALLED ||
                         robot_state == actionlib::SimpleClientGoalState::REJECTED ||
                         robot_state == actionlib::SimpleClientGoalState::PREEMPTED ||
                         robot_state == actionlib::SimpleClientGoalState::ABORTED) {
                if (robot_command_status_[robot_idx] == GOING_TO_HELP_DESTINATION_LOCATION) {
                  robot_command_status_[robot_idx] = HELP_DESTINATION_NAVIGATION_FAILED;
                } else if (robot_command_status_[robot_idx] == GOING_TO_SERVICE_TASK_LOCATION) {
                  robot_command_status_[robot_idx] = SERVICE_TASK_NAVIGATION_RESET;
                }
                // else {
                //   ROS_FATAL_STREAM("BAD STATE TRANSITION 2 - robot failed navigation action in command status" << robot_command_status_[robot_idx]);
                //   throw std::runtime_error("");
                // }
              }
            }
          }


          if (all_robot_locations_available) {
            bool its_decision_time = false;
            if (episode_in_progress_ && !terminate_episode_) {
              if (at_episode_start_) {
                // Ensure that all robots switch to a scenario where they are no longer
                bwi_guidance_msgs::UpdateGuidanceGui srv;
                srv.request.type = bwi_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_NOTHING;
                for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
                  robot_gui_controller_[robot_idx]->call(srv);
                }

                // Determine the human's start location.
                int loc;
                determineStartLocation(human_location_, loc);
                system_state_.loc_node = system_state_.loc_prev = loc;
                system_state_.assist_loc = NONE;
                system_state_.assist_type = NONE;
                ROS_INFO_STREAM_NAMED("base_robot_navigator", "System state at start determined: " << system_state_);
                wait_start_state_ = system_state_;
                mcts_->restart();
                compute(0.1f);
                at_episode_start_ = false;
                its_decision_time = true;
              } else {
                int next_loc;
                determineHumanTransitionalLocation(human_location_, system_state_.loc_node, next_loc);
                if (next_loc != system_state_.loc_node) {
                  if (next_loc == goal_node_id_) {
                    ROS_INFO_STREAM("Reached destination!");
                    // TODO maybe do something special here if a robot is leading the person.
                    for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
                      if (system_state_.robots[robot_idx].help_destination != NONE) {
                        system_state_.robots[robot_idx].help_destination = NONE;
                      }
                    }
                    episode_completed_ = true;
                    episode_in_progress_ = false;
                  } else {
                    system_state_.loc_prev = system_state_.loc_node;
                    system_state_.loc_node = next_loc;
                    system_state_.assist_type = NONE;
                    system_state_.assist_loc = NONE;
                    wait_start_state_ = system_state_;
                    mcts_->restart();
                    compute(0.1f);
                    its_decision_time = true;
                  }
                }
              }
              publishCurrentSystemState();
            } else if (terminate_episode_) {
              terminate_episode_ = false;
              for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
                if (system_state_.robots[robot_idx].help_destination != NONE) {
                  system_state_.robots[robot_idx].help_destination = NONE;
                }
              }
            }

            if (its_decision_time) {
              while (true) {
                Action action = getBestAction();
                ROS_INFO_STREAM_NAMED("base_robot_navigator", "taking action " << action);
                if (action.type != WAIT) {
                  // See if the action requires some interaction with the GUI.
                  if (action.type == DIRECT_PERSON) {
                    pause_robot_ = action.robot_id;
                    bwi_guidance_msgs::UpdateGuidanceGui srv;
                    srv.request.type = bwi_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_ORIENTATION;
                    robot_gui_controller_[action.robot_id]->call(srv);
                  } else if (action.type == LEAD_PERSON) {
                    pause_robot_ = action.robot_id;
                    bwi_guidance_msgs::UpdateGuidanceGui srv;
                    srv.request.type = bwi_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_FOLLOWME;
                    robot_gui_controller_[action.robot_id]->call(srv);
                    robot_command_status_[action.robot_id] = SERVICE_TASK_NAVIGATION_RESET;
                  }
                  // Perform the deterministic transition as per the model
                  ExtendedState next_state;
                  bool unused_terminal; /* The resulting state can never be terminal via a non WAIT action */
                  float unused_reward_value;
                  int unused_depth_count;

                  // Note that the RNG won't be used as it is a deterministic action.
                  model_->takeAction(system_state_, action, unused_reward_value, next_state, unused_terminal, unused_depth_count, master_rng_);
                  system_state_ = next_state;
                } else {
                  // Let's switch to non-deterministic transition logic.
                  wait_start_state_ = system_state_;
                  wait_action_start_time_ = boost::posix_time::microsec_clock::local_time();
                  break;
                }
              }
            }

            for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
              RobotState &rs = system_state_.robots[robot_idx];

              // Check if a robot service task is still being initialized, or was just completed.
              if (robot_command_status_[robot_idx] == INITIALIZED) {
                robot_command_status_[robot_idx] = SERVICE_TASK_NAVIGATION_RESET;
                if (!episode_in_progress_) {
                  bwi_guidance_msgs::UpdateGuidanceGui srv;
                  srv.request.type = bwi_guidance_msgs::UpdateGuidanceGuiRequest::DISABLE_EPISODE_START;
                  robot_gui_controller_[robot_idx]->call(srv);
                }
              } else if (robot_command_status_[robot_idx] == AT_SERVICE_TASK_LOCATION) {
                if (rs.tau_t == 0.0f) {
                  ROS_DEBUG_STREAM_NAMED("BaseRobotNavigator", available_robot_list_[robot_idx] << " reached service task location.");
                  rs.tau_t = 1.0f / controller_thread_frequency_;
                  // The second term tries to average for discretization errors. TODO think about this some more when
                  // not sleepy.
                  robot_service_task_start_time_[robot_idx] = boost::posix_time::microsec_clock::local_time() -
                    boost::posix_time::milliseconds(0.5 * 1.0f / controller_thread_frequency_);
                } else {
                  boost::posix_time::time_duration diff =
                    boost::posix_time::microsec_clock::local_time() - robot_service_task_start_time_[robot_idx];
                  rs.tau_t = diff.total_milliseconds() / 1000.0f;
                  ROS_DEBUG_STREAM_NAMED("BaseRobotNavigator", available_robot_list_[robot_idx] << " has been at service task location for " << rs.tau_t << " seconds.");
                }
                if (rs.tau_t > rs.tau_total_task_time) {
                  // This service task is now complete. Get a new task.
                  getNextTaskForRobot(robot_idx, rs);
                  robot_command_status_[robot_idx] = SERVICE_TASK_NAVIGATION_RESET;
                  if (!episode_in_progress_) {
                    bwi_guidance_msgs::UpdateGuidanceGui srv;
                    srv.request.type = bwi_guidance_msgs::UpdateGuidanceGuiRequest::DISABLE_EPISODE_START;
                    robot_gui_controller_[robot_idx]->call(srv);
                  }
                }
              }

              // Check if a robot that was manually paused no longer needs to be paused.
              if (pause_robot_ == robot_idx) {
                boost::posix_time::time_duration time_since_wait_start =
                  boost::posix_time::microsec_clock::local_time() - wait_action_start_time_;
                if (time_since_wait_start.total_milliseconds() > 3000) {
                  pause_robot_ = NONE;
                  bwi_guidance_msgs::UpdateGuidanceGui srv;
                  srv.request.type = bwi_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_NOTHING;
                  robot_gui_controller_[robot_idx]->call(srv);
                }
              } else {
                bool robot_aiding_human = rs.help_destination != NONE;
                if (robot_aiding_human) {
                  if (robot_command_status_[robot_idx] != AT_HELP_DESTINATION_LOCATION &&
                      robot_command_status_[robot_idx] != GOING_TO_HELP_DESTINATION_LOCATION) {
                    int destination = rs.help_destination;
                    float orientation = bwi_mapper::getNodeAngle(system_state_.loc_node, rs.help_destination, graph_);
                    sendRobotToDestination(robot_idx, destination, orientation);
                    robot_command_status_[robot_idx] = GOING_TO_HELP_DESTINATION_LOCATION;
                  } /* TODO see if the robot needs to be rotated. */
                } else {
                  if (robot_command_status_[robot_idx] != AT_SERVICE_TASK_LOCATION &&
                      robot_command_status_[robot_idx] != GOING_TO_SERVICE_TASK_LOCATION) {
                    int destination = rs.tau_d;
                    float orientation = bwi_mapper::getNodeAngle(rs.loc_u, rs.tau_d, graph_);
                    sendRobotToDestination(robot_idx, destination, orientation);
                    robot_command_status_[robot_idx] = GOING_TO_SERVICE_TASK_LOCATION;
                  }
                }
              }
            }
          } else {
            ROS_WARN_THROTTLE_NAMED(1.0f, "MultiRobotNavigator", " still waiting for robot locations. This shouldn't take too long...");
          }
        }

        if (!episode_in_progress_) {
          boost::this_thread::sleep(boost::posix_time::milliseconds(1000.0f/controller_thread_frequency_));
        } else {
          compute(1.0f/controller_thread_frequency_);
        }

      }
    }

    void BaseRobotNavigator::publishCurrentSystemState() {
      cv::Mat img = base_image_.clone();
      model_->drawState(system_state_, img);
      cv::imshow("out", img);
    }

    void BaseRobotNavigator::sendRobotToDestination(int robot_idx, int destination, float orientation) {
      bwi_mapper::Point2f dest = getLocationFromGraphId(destination);
      move_base_msgs::MoveBaseGoal goal;
      geometry_msgs::PoseStamped &target_pose = goal.target_pose;
      target_pose.header.stamp = ros::Time::now();
      target_pose.header.frame_id = available_robot_list_[robot_idx] + "/level_mux/map";
      target_pose.pose.position.x = dest.x;
      target_pose.pose.position.y = dest.y;
      target_pose.pose.position.z = 0;
      target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(orientation);
      ROS_INFO_STREAM("Sending " << available_robot_list_[robot_idx] << " to " << destination << " (" << dest.x << "," << dest.y << ")");
      robot_controller_[robot_idx]->sendGoal(goal);
    }

    bwi_mapper::Point2f BaseRobotNavigator::getLocationFromGraphId(int destination) {
      bwi_mapper::Point2f dest = bwi_mapper::getLocationFromGraphId(destination, graph_);
      return bwi_mapper::toMap(dest, map_.info);
    }

    void BaseRobotNavigator::determineHumanTransitionalLocation(const geometry_msgs::Pose &pose,
                                                                int current_loc,
                                                                int& next_loc) {
      bwi_mapper::Point2f human_pt(pose.position.x, pose.position.y);
      bwi_mapper::Point2f human_grid_pt = bwi_mapper::toGrid(human_pt, map_.info);
      int next_graph_id = bwi_mapper::getClosestEdgeOnGraphGivenId(human_grid_pt, graph_, current_loc);
      bwi_mapper::Point2f current_pt = getLocationFromGraphId(current_loc);
      bwi_mapper::Point2f next_pt = getLocationFromGraphId(next_graph_id);
      if (bwi_mapper::getMagnitude(next_pt - human_pt) <= 1.5f) {
        next_loc = next_graph_id;
      } else {
        next_loc = current_loc;
      }
    }

    void BaseRobotNavigator::determineStartLocation(const geometry_msgs::Pose &pose, int &u, int &v, float &p) {
      bwi_mapper::Point2f pt(pose.position.x, pose.position.y);
      bwi_mapper::Point2f pt_grid = bwi_mapper::toGrid(pt, map_.info);
      u = bwi_mapper::getClosestIdOnGraph(pt_grid, graph_);
      v = bwi_mapper::getClosestEdgeOnGraphGivenId(pt_grid, graph_, u);
      bwi_mapper::Point2f u_loc = getLocationFromGraphId(u);
      bwi_mapper::Point2f v_loc = getLocationFromGraphId(v);
      p = bwi_mapper::getMagnitude(pt - u_loc) / (bwi_mapper::getMagnitude(pt - u_loc) + bwi_mapper::getMagnitude(pt - v_loc));
    }

    void BaseRobotNavigator::determineStartLocation(const geometry_msgs::Pose &pose, int &u) {
      bwi_mapper::Point2f pt(pose.position.x, pose.position.y);
      bwi_mapper::Point2f pt_grid = bwi_mapper::toGrid(pt, map_.info);
      u = bwi_mapper::getClosestIdOnGraph(pt_grid, graph_);
    }

    void BaseRobotNavigator::determineRobotTransitionalLocation(const geometry_msgs::Pose &pose, RobotState &rs) {
      // Change loc_v everytime, but keep loc_u constant.
      bwi_mapper::Point2f pt(pose.position.x, pose.position.y);
      bwi_mapper::Point2f pt_grid = bwi_mapper::toGrid(pt, map_.info);
      rs.loc_v = bwi_mapper::getClosestEdgeOnGraphGivenId(pt_grid, graph_, rs.loc_u);
      bwi_mapper::Point2f u_loc = getLocationFromGraphId(rs.loc_u);
      bwi_mapper::Point2f v_loc = getLocationFromGraphId(rs.loc_v);
      rs.loc_p = bwi_mapper::getMagnitude(pt - u_loc) /
          (bwi_mapper::getMagnitude(pt - u_loc) + bwi_mapper::getMagnitude(pt - v_loc));
      // Swapping allows the robot to transition from one node to the next.
      if (rs.loc_p >= 0.6f) {
        int temp = rs.loc_u;
        rs.loc_u = rs.loc_v;
        rs.loc_v = temp;
        rs.loc_p = 1 - rs.loc_p;
      }
    }

    void BaseRobotNavigator::roundOffRobotLocation(RobotState &rs) {
      if (rs.loc_p >= 0.5f) {
        rs.loc_p = 0.0f;
        int temp = rs.loc_v;
        rs.loc_u = rs.loc_v;
        rs.loc_v = temp;
      } else {
        // u and v are correctly set anyway!
        rs.loc_p = 0.0f;
      }
    }

    Action BaseRobotNavigator::getBestAction() {
      return mcts_->selectWorldAction(system_state_);
    }

    void BaseRobotNavigator::getNextTaskForRobot(int robot_id, RobotState &rs) {
      task_generation_model_->generateNewTaskForRobot(robot_id, rs, *master_rng_);
    }

    void BaseRobotNavigator::compute(float timeout) {
      unsigned int unused_rollout_termination_count;
      mcts_->search(wait_start_state_, unused_rollout_termination_count, timeout);
    }

  } /* mrn */

} /* bwi_guidance_solver */
