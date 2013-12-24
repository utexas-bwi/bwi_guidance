#include<fstream>

#include <bwi_guidance_solver/ValueIteration.h>
#include <bwi_guidance_solver/person_estimator2.h>
#include <bwi_guidance_solver/person_model2.h>
#include <bwi_guidance_solver/heuristic_solver.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_guidance/base_robot_positioner.h>
#include <tf/transform_datatypes.h>
#include <boost/foreach.hpp>

using namespace bwi_guidance;

class VIRobotPositioner2 : public BaseRobotPositioner {

  private:
    boost::shared_ptr<PersonModel2> model_;
    boost::shared_ptr<PersonEstimator2> estimator_;
    boost::shared_ptr<ValueIteration<State, Action> > vi_;
    boost::shared_ptr<HeuristicSolver> hs_;

    std::map<int, boost::shared_ptr<PersonModel2> > model_map_;
    std::map<int, boost::shared_ptr<PersonEstimator2> > estimator_map_;
    std::map<int, boost::shared_ptr<ValueIteration<State, Action> > > vi_map_;
    std::map<int, boost::shared_ptr<HeuristicSolver> > hs_map_;

    double vi_gamma_;
    int vi_max_iterations_;
    std::string data_directory_;
    bool use_heuristic_;
    bool allow_robot_current_idx_;
    double visibility_range_;
    bool allow_goal_visibility_;

    std::string instance_name_;
    int goal_idx_;
    State current_state_;

    size_t assigned_robots_;
    bwi_mapper::Point2f assigned_robot_loc_;
    float assigned_robot_yaw_;
    std::map<int, int> graph_id_to_robot_map_;

  public:

    VIRobotPositioner2(boost::shared_ptr<ros::NodeHandle>& nh) :
        BaseRobotPositioner(nh) {

      ros::NodeHandle private_nh("~");
      private_nh.param<std::string>("data_directory", data_directory_, "");
      private_nh.param<double>("vi_gamma", vi_gamma_, 1.0);
      private_nh.param<int>("vi_max_iterations", vi_max_iterations_, 1000);
      private_nh.param<bool>("use_heuristic", use_heuristic_, false);
      private_nh.param<bool>("allow_robot_current", allow_robot_current_idx_, 
          false);
      private_nh.param<bool>("allow_goal_visibility", allow_goal_visibility_, 
          true);
      private_nh.param<double>("visibility_range", visibility_range_, 
          30.0);

      if (use_heuristic_)
        ROS_INFO_STREAM("Using heuristic!");
      else
        ROS_INFO_STREAM("Using VI!");
      ROS_INFO_STREAM("Allow placing robot at current location: " <<
          allow_robot_current_idx_);
      ROS_INFO_STREAM("Simulator visibility: " << visibility_range_);
      ROS_INFO_STREAM("Allor visibility of goal: " << allow_goal_visibility_);

      // Pre-compute all the experiment related information
      std::vector<std::string> instance_names;
      getInstanceNames(experiment_, instance_names);
      BOOST_FOREACH(const std::string iname, instance_names) {
        const Instance& instance = getInstance(experiment_, iname);
        bwi_mapper::Point2f goal_point(instance.ball_loc.x,
            instance.ball_loc.y);
        goal_point = bwi_mapper::toGrid(goal_point, map_info_);
        int goal_idx = 
          bwi_mapper::getClosestIdOnGraph(goal_point, graph_);

        // Compute model file
        std::string model_file = data_directory_
          + boost::lexical_cast<std::string>(goal_idx) + "_model.txt";
        std::string vi_file = data_directory_
          + boost::lexical_cast<std::string>(goal_idx) + "_vi.txt";

        // Setup the model and the heuristic solver to read from file
        boost::shared_ptr<PersonModel2> model;
        boost::shared_ptr<PersonEstimator2> estimator;
        boost::shared_ptr<ValueIteration<State, Action> > vi;
        boost::shared_ptr<HeuristicSolver> hs;
        float pixel_visibility_range = visibility_range_ / map_.info.resolution;
        model.reset(new PersonModel2(graph_, map_, goal_idx, model_file,
              allow_robot_current_idx_, pixel_visibility_range,
              allow_goal_visibility_));
        estimator.reset(new PersonEstimator2);
        float epsilon = 0.05f / map_.info.resolution;
        float delta = -500.0f / map_.info.resolution;
        vi.reset(new ValueIteration<State, Action>(
              model, estimator, vi_gamma_, epsilon, vi_max_iterations_,
              0.0, delta));
        hs.reset(new HeuristicSolver(map_, graph_, goal_idx,
              allow_robot_current_idx_, pixel_visibility_range, 
              allow_goal_visibility_)); 

        bool policy_available = false;
        std::ifstream fin(vi_file.c_str());
        if (fin.good()) {
          policy_available = true;
        }
        if (policy_available) {
          vi->loadPolicy(vi_file);
          ROS_INFO_STREAM("VIRobotPositioner2: Loaded policy for goal_idx " <<
              goal_idx << " from " << vi_file);
        } else {
          ROS_INFO_STREAM("VIRobotPositioner2: Computing policy for goal_idx: "
              << goal_idx);
          vi->computePolicy();
          vi->savePolicy(vi_file);
          ROS_INFO_STREAM("VIRobotPositioner2: Saved policy to " << vi_file);
        }

        model_map_[goal_idx] = model;
        estimator_map_[goal_idx] = estimator;
        vi_map_[goal_idx] = vi;
        hs_map_[goal_idx] = hs;
      }

    }

    virtual ~VIRobotPositioner2() {}

    virtual void startExperimentInstance(
        const std::string& instance_name) {

      instance_name_ = instance_name;
      assigned_robots_ = 0;

      const Instance& instance = getInstance(experiment_, instance_name_);

      bwi_mapper::Point2f start_point(instance.start_loc.x,
          instance.start_loc.y);
      start_point = bwi_mapper::toGrid(start_point, map_info_);
      size_t start_idx = 
        bwi_mapper::getClosestIdOnGraph(start_point, graph_);

      bwi_mapper::Point2f goal_point(instance.ball_loc.x,
          instance.ball_loc.y);
      goal_point = bwi_mapper::toGrid(goal_point, map_info_);
      goal_idx_ = 
        bwi_mapper::getClosestIdOnGraph(goal_point, graph_);
 
      model_ = model_map_[goal_idx_];
      estimator_ = estimator_map_[goal_idx_];
      vi_ = vi_map_[goal_idx_];
      hs_ = hs_map_[goal_idx_];

      size_t direction = 
        getDiscretizedAngle(instance.start_loc.yaw);

      current_state_.graph_id = start_idx;
      current_state_.direction = direction;
      current_state_.num_robots_left = instance.max_robots;
      current_state_.robot_direction = NONE;
      current_state_.visible_robot = NONE;
      ROS_INFO_STREAM("Start at: " << current_state_);
      checkRobotPlacementAtCurrentState();
    }

    virtual void checkRobotPlacementAtCurrentState() {

      boost::mutex::scoped_lock lock(robot_modification_mutex_);

      const Instance& instance = 
        getInstance(experiment_, instance_name_);

      // First check if we need to place a robot according to VI policy
      Action action;
      if (use_heuristic_) {
        action = hs_->getBestAction(current_state_);
      } else {
        action = vi_->getBestAction(current_state_);
      }
      std::vector<State> next_states;
      std::vector<float> probabilities;
      std::vector<float> rewards;
      model_->getTransitionDynamics(current_state_, action, next_states, 
          rewards, probabilities);

      while (action.type != DO_NOTHING) {
        current_state_ = next_states[0];
        ROS_INFO_STREAM("AUTO transition to: " << current_state_);
        if (action.type == DIRECT_PERSON) {

          // Figure out direction to point towards here
          bwi_mapper::Point2f to_loc = 
            bwi_mapper::getLocationFromGraphId(
                current_state_.robot_direction, graph_);
          bwi_mapper::Point2f change_loc = to_loc - assigned_robot_loc_;
          float destination_yaw = atan2(change_loc.y, change_loc.x);
          float change_in_yaw = destination_yaw - assigned_robot_yaw_;
          cv::Mat robot_image;
          produceDirectedArrow(change_in_yaw, robot_image);

          // Teleport the robot and assign a direction
          std::string robot_id = default_robots_.robots[assigned_robots_ - 1].id;
          robot_screen_orientations_[robot_id] = destination_yaw;
          robot_screen_publisher_.updateImage(robot_id, robot_image);

        } else {

          // Place a robot here. Use lookahead to determine the best positioning
          // around the node
          bwi_mapper::Point2f at_loc = 
            bwi_mapper::getLocationFromGraphId(
                current_state_.visible_robot, graph_);

          float angle = bwi_mapper::getNodeAngle(
                current_state_.graph_id, 
                current_state_.visible_robot, graph_
                );
          bwi_mapper::Point2f from_loc =
            at_loc - bwi_mapper::Point2f(
                50.0 * cosf(angle),
                50.0 * sinf(angle));

          // Perform lookahead to see best location to place robot
          size_t direction_idx = getDiscretizedAngle(angle);
          State robot_state;
          robot_state.graph_id = current_state_.visible_robot;
          robot_state.direction = direction_idx;
          robot_state.num_robots_left = current_state_.num_robots_left;
          robot_state.robot_direction = DIR_UNASSIGNED;
          robot_state.visible_robot = NONE;
          Action robot_action;
          if (use_heuristic_) {
            robot_action = hs_->getBestAction(robot_state);
          } else {
            robot_action = vi_->getBestAction(robot_state);
          }
          bwi_mapper::Point2f to_loc = 
            bwi_mapper::getLocationFromGraphId(
                robot_action.graph_id, graph_);

          // Compute robot pose
          geometry_msgs::Pose pose = positionRobot(from_loc, at_loc, to_loc);
          assigned_robot_yaw_ = tf::getYaw(pose.orientation);
          assigned_robot_loc_ = 
            bwi_mapper::Point2f(pose.position.x, pose.position.y);
          assigned_robot_loc_ = 
            bwi_mapper::toGrid(assigned_robot_loc_, map_info_);

          // Teleport the robot and forward this information
          std::string robot_id = default_robots_.robots[assigned_robots_].id;
          robot_locations_[robot_id] = pose;
          graph_id_to_robot_map_[current_state_.visible_robot] =
            assigned_robots_;

          ++assigned_robots_;
        }
        if (use_heuristic_) {
          action = hs_->getBestAction(current_state_);
        } else {
          action = vi_->getBestAction(current_state_);
        }
        model_->getTransitionDynamics(current_state_, action, next_states, 
            rewards, probabilities);
      }
    }

    virtual void odometryCallback(const nav_msgs::Odometry::ConstPtr odom) {
      
      if (!instance_in_progress_)
        return;

      bwi_mapper::Point2f person_loc(
          odom->pose.pose.position.x,
          odom->pose.pose.position.y);
      person_loc = bwi_mapper::toGrid(person_loc, map_info_);

      size_t current_graph_id =
        bwi_mapper::getClosestIdOnGraphFromEdge(person_loc,
            graph_, current_state_.graph_id);

      if (current_graph_id != current_state_.graph_id) {
        // A transition has happened. Compute next state and check if robot 
        // needs to be placed.
        Action a(DO_NOTHING, 0);
        std::vector<State> next_states; 
        model_->getNextStates(current_state_, a, next_states); 
        int old_robot_status = current_state_.robot_direction;
        if (old_robot_status != NONE) {
          boost::mutex::scoped_lock lock(robot_modification_mutex_);
          int robot_number = 
            graph_id_to_robot_map_[current_state_.graph_id];
          std::string old_robot_id = 
            default_robots_.robots[robot_number].id;
          robot_locations_[old_robot_id] = 
            convert2dToPose(
                default_robots_.robots[robot_number].default_loc.x,
                default_robots_.robots[robot_number].default_loc.y,
                0);
        }

        BOOST_FOREACH(const State& state, next_states) {
          if (state.graph_id == current_graph_id) {
            int old_robot_state = current_state_.visible_robot;
            current_state_ = state;
            if (old_robot_state != NONE && 
                current_state_.visible_robot == NONE &&
                current_state_.robot_direction == NONE) {
              int robot_number = 
                graph_id_to_robot_map_[old_robot_state];
              std::string old_robot_id = 
                default_robots_.robots[robot_number].id;
              robot_locations_[old_robot_id] = 
                convert2dToPose(
                    default_robots_.robots[robot_number].default_loc.x,
                    default_robots_.robots[robot_number].default_loc.y,
                    0);
            }
            ROS_INFO_STREAM("MANUAL transition to: " << current_state_);
            checkRobotPlacementAtCurrentState();
            break;
          }
        }
      }
    }

};


int main(int argc, char** argv) {

  ros::init(argc, argv, "vi_robot_positioner2");
  boost::shared_ptr<ros::NodeHandle> nh;
  nh.reset(new ros::NodeHandle());
  VIRobotPositioner2 rp(nh);
  rp.run();

  return 0;
}
