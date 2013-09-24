#include<fstream>

#include <bwi_exp1_solver/ValueIteration.h>
#include <bwi_exp1_solver/person_estimator2.h>
#include <bwi_exp1_solver/person_model2.h>
#include <topological_mapper/map_loader.h>
#include <bwi_exp1/base_robot_positioner.h>
#include <tf/transform_datatypes.h>
#include <boost/foreach.hpp>

using namespace bwi_exp1;

class VIRobotPositioner2 : public BaseRobotPositioner {

  private:
    boost::shared_ptr<PersonModel2> model_;
    boost::shared_ptr<PersonEstimator2> estimator_;
    boost::shared_ptr<ValueIteration<State2, Action> > vi_;

    std::string policy_file_;
    int num_robots_available_;
    int goal_idx_;
    double vi_gamma_;
    int vi_max_iterations_;
    bool recompute_policy_;

    std::string instance_name_;
    State2 current_state_;

    size_t assigned_robots_;
    topological_mapper::Point2f assigned_robot_loc_;
    float assigned_robot_yaw_;
    std::map<int, int> graph_id_to_robot_map_;

  public:

    VIRobotPositioner2(boost::shared_ptr<ros::NodeHandle>& nh) :
        BaseRobotPositioner(nh) {

      ros::NodeHandle private_nh("~");
      private_nh.getParam("policy_file", policy_file_);
      private_nh.getParam("goal_idx", goal_idx_);
      private_nh.param("num_robots_available", num_robots_available_, 5);
      private_nh.param<bool>("recompute_policy", recompute_policy_, false);
      private_nh.param<double>("vi_gamma", vi_gamma_, 1.0);
      private_nh.param<int>("vi_max_iterations_", vi_max_iterations_, 1000);

      model_.reset(new PersonModel2(graph_, map_, goal_idx_));
      estimator_.reset(new PersonEstimator2);
      vi_.reset(new ValueIteration<State2, Action>(
            model_, estimator_, vi_gamma_, 1.0, vi_max_iterations_));

      bool policyAvailable = false;
      std::ifstream fin(policy_file_.c_str());
      if (fin.good()) {
        policyAvailable = true;
      }
      if (policyAvailable && !recompute_policy_) {
        vi_->loadPolicy(policy_file_);
        ROS_INFO_STREAM("VIRobotPositioner2: Read policy from file: " << 
            policy_file_);
      } else {
        vi_->computePolicy();
        vi_->savePolicy(policy_file_);
        ROS_INFO_STREAM("VIRobotPositioner2: Computed policy and saved it " <<
            "to file: " <<  policy_file_);
      }

    }

    virtual ~VIRobotPositioner2() {}

    virtual void startExperimentInstance(
        const std::string& instance_name) {

      instance_name_ = instance_name;
      assigned_robots_ = 0;

      const Instance& instance = 
        getInstance(experiment_, instance_name_);

      topological_mapper::Point2f start_point(instance.start_loc.x,
          instance.start_loc.y);
      start_point = topological_mapper::toGrid(start_point, map_info_);

      size_t prev_graph_id = 
        topological_mapper::getClosestIdOnGraph(start_point, graph_);
      size_t direction = 
        model_->getDirectionFromAngle(instance.start_loc.yaw);

      current_state_.graph_id = prev_graph_id;
      current_state_.direction = direction;
      current_state_.num_robots_left = num_robots_available_;
      current_state_.current_robot_status = NO_ROBOT;
      current_state_.visible_robot_location = NO_ROBOT;
      ROS_INFO_STREAM("Start at: " << current_state_);
      checkRobotPlacementAtCurrentState();
    }

    virtual void checkRobotPlacementAtCurrentState(
        size_t prev_graph_id = (size_t)-1) {

      boost::mutex::scoped_lock lock(robot_modification_mutex_);

      const Instance& instance = 
        getInstance(experiment_, instance_name_);

      // First check if we need to place a robot according to VI policy
      Action action = vi_->getBestAction(current_state_);
      std::vector<State2> next_states;
      std::vector<float> probabilities;
      std::vector<float> rewards;
      model_->getTransitionDynamics(current_state_, action, next_states, 
          rewards, probabilities);

      while (action.type != DO_NOTHING) {
        current_state_ = next_states[0];
        ROS_INFO_STREAM("AUTO transition to: " << current_state_);
        if (action.type == DIRECT_PERSON) {
          // DIRECT ROBOT HERE
          topological_mapper::Point2f to_loc = 
            topological_mapper::getLocationFromGraphId(
                current_state_.current_robot_status, graph_);
          topological_mapper::Point2f change_loc = to_loc - assigned_robot_loc_;
          float destination_yaw = atan2(change_loc.y, change_loc.x);
          float change_in_yaw = destination_yaw - assigned_robot_yaw_;
          ROS_INFO_STREAM("  " << to_loc << " " << assigned_robot_loc_ << " " << assigned_robot_yaw_);
          cv::Mat robot_image;
          produceDirectedArrow(change_in_yaw, robot_image);

          // Teleport the robot and assign a direction
          std::string robot_id = default_robots_.robots[assigned_robots_ - 1].id;
          robot_screen_orientations_[robot_id] = destination_yaw;
          robot_screen_publisher_.updateImage(robot_id, robot_image);
        } else {
          // PLACE ROBOT HERE
          assigned_robot_loc_ = 
            topological_mapper::getLocationFromGraphId(
                current_state_.visible_robot_location, graph_);
          topological_mapper::Point2f robot_map_loc =
            topological_mapper::toMap(assigned_robot_loc_, map_.info);
          geometry_msgs::Pose pose;
          pose.position.x = robot_map_loc.x;
          pose.position.y = robot_map_loc.y;
          pose.position.z = 0;
          assigned_robot_yaw_ = 
            model_->getAngleFromStates(
                current_state_.graph_id, 
                current_state_.visible_robot_location
                );
          pose.orientation =
            tf::createQuaternionMsgFromYaw(assigned_robot_yaw_);
          std::string robot_id = default_robots_.robots[assigned_robots_].id;
          robot_locations_[robot_id] = pose;
          graph_id_to_robot_map_[current_state_.visible_robot_location] =
            assigned_robots_;

          ++assigned_robots_;
        }
        action = vi_->getBestAction(current_state_);
        model_->getTransitionDynamics(current_state_, action, next_states, 
            rewards, probabilities);
      }
    }

    virtual void odometryCallback(const nav_msgs::Odometry::ConstPtr odom) {
      
      if (!instance_in_progress_)
        return;

      topological_mapper::Point2f person_loc(
          odom->pose.pose.position.x,
          odom->pose.pose.position.y);
      person_loc = topological_mapper::toGrid(person_loc, map_info_);

      size_t current_graph_id =
        topological_mapper::getClosestIdOnGraphFromEdge(person_loc,
            graph_, current_state_.graph_id);

      if (current_graph_id != current_state_.graph_id) {
        // A transition has happened. Compute next state and check if robot 
        // needs to be placed.
        Action a(DO_NOTHING, 0);
        std::vector<State2> next_states; 
        model_->getNextStates(current_state_, a, next_states); 
        int old_robot_status = current_state_.current_robot_status;
        if (old_robot_status != NO_ROBOT) {
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

        BOOST_FOREACH(const State2& state, next_states) {
          if (state.graph_id == current_graph_id) {
            int old_robot_state = current_state_.visible_robot_location;
            current_state_ = state;
            if (old_robot_state != NO_ROBOT && 
                current_state_.visible_robot_location == NO_ROBOT &&
                current_state_.current_robot_status == NO_ROBOT) {
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
