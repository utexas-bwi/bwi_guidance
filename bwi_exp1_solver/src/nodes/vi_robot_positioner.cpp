#include<fstream>

#include <bwi_exp1_solver/ValueIteration.h>
#include <bwi_exp1_solver/person_estimator.h>
#include <bwi_exp1_solver/person_model.h>
#include <topological_mapper/map_loader.h>
#include <bwi_exp1/base_robot_positioner.h>
#include <tf/transform_datatypes.h>

using namespace bwi_exp1;

class VIRobotPositioner : public BaseRobotPositioner {

  private:
    boost::shared_ptr<PersonModel> model_;
    boost::shared_ptr<PersonEstimator> estimator_;
    boost::shared_ptr<ValueIteration<state_t, action_t> > vi_;

    std::string policy_file_;
    int num_robots_available_;
    int goal_idx_;
    double vi_gamma_;
    int vi_max_iterations_;
    bool recompute_policy_;

    std::string instance_name_;
    state_t current_state_;
    unsigned int robots_at_next_state_;
    size_t assigned_robots_;

  public:

    VIRobotPositioner(boost::shared_ptr<ros::NodeHandle>& nh) :
        BaseRobotPositioner(nh) {

      ros::NodeHandle private_nh("~");
      private_nh.getParam("policy_file", policy_file_);
      private_nh.getParam("goal_idx", goal_idx_);
      private_nh.param("num_robots_available", num_robots_available_, 5);
      private_nh.param<bool>("recompute_policy", recompute_policy_, false);
      private_nh.param<double>("vi_gamma", vi_gamma_, 0.98);
      private_nh.param<int>("vi_max_iterations_", vi_max_iterations_, 1000);

      model_.reset(new PersonModel(graph_, goal_idx_));
      estimator_.reset(new PersonEstimator(model_->getStateSpaceSize(), 0));
      vi_.reset(new ValueIteration<state_t, action_t>(
            model_, estimator_, vi_gamma_, vi_max_iterations_));

      bool policyAvailable = false;
      std::ifstream fin(policy_file_.c_str());
      if (fin.good()) {
        policyAvailable = true;
      }
      if (policyAvailable && !recompute_policy_) {
        vi_->loadPolicy(policy_file_);
        ROS_INFO_STREAM("VIRobotPositioner: Read policy from file: " << 
            policy_file_);
      } else {
        vi_->computePolicy();
        vi_->savePolicy(policy_file_);
        ROS_INFO_STREAM("VIRobotPositioner: Computed policy and saved it " <<
            "to file: " <<  policy_file_);
      }

    }

    virtual ~VIRobotPositioner() {}

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

      ROS_INFO_STREAM("Start: " << prev_graph_id << 
          ", direction: " << direction << 
          ", num_robots: " << num_robots_available_);
      current_state_ = model_->canonicalizeState(prev_graph_id, 
          direction, num_robots_available_);
      checkRobotPlacementAtCurrentState();

    }

    virtual void checkRobotPlacementAtCurrentState(
        size_t prev_graph_id = (size_t)-1) {

      boost::mutex::scoped_lock lock(robot_modification_mutex_);

      const Instance& instance = 
        getInstance(experiment_, instance_name_);

      // First check if we need to place a robot according to VI policy
      State current_state = model_->resolveState(current_state_);

      action_t action_idx = vi_->getBestAction(current_state_);
      Action action = model_->resolveAction(current_state_, action_idx);
      if (action.type == PLACE_ROBOT) {

        topological_mapper::Point2f at_loc = 
          topological_mapper::getLocationFromGraphId(
              current_state.graph_id, graph_);

        topological_mapper::Point2f from_loc(
            instance.start_loc.x,
            instance.start_loc.y);
        from_loc = topological_mapper::toGrid(from_loc, map_info_);
        if (prev_graph_id != (size_t)-1) {
          // A prev_idx is available, use it instead of the starting location
          from_loc = topological_mapper::getLocationFromGraphId(
              prev_graph_id, graph_);
        }

        topological_mapper::Point2f to_loc = 
          topological_mapper::getLocationFromGraphId(
              action.graph_id, graph_);

        // Compute robot pose
        geometry_msgs::Pose pose = positionRobot(from_loc, at_loc, to_loc);
        float robot_yaw = tf::getYaw(pose.orientation);

        // Compute arrow pose
        topological_mapper::Point2f robot_loc(pose.position.x, pose.position.y);
        robot_loc = topological_mapper::toGrid(robot_loc, map_info_);
        topological_mapper::Point2f change_loc = to_loc - robot_loc;
        float destination_yaw = atan2(change_loc.y, change_loc.x);
        float change_in_yaw = destination_yaw - robot_yaw;
        cv::Mat robot_image;
        produceDirectedArrow(change_in_yaw, robot_image);

        // Teleport the robot and assign a direction
        std::string robot_id = default_robots_.robots[assigned_robots_].id;
        robot_locations_[robot_id] = pose;
        robot_screen_orientations_[robot_id] = destination_yaw;
        robot_screen_publisher_.updateImage(robot_id, robot_image);
        ++assigned_robots_;
        robots_at_next_state_ = current_state.num_robots_left - 1;
      } else {
        robots_at_next_state_ = current_state.num_robots_left;
      }


    }

    virtual void odometryCallback(const nav_msgs::Odometry::ConstPtr odom) {
      
      if (!instance_in_progress_)
        return;

      State current_state = model_->resolveState(current_state_);

      topological_mapper::Point2f person_loc(
          odom->pose.pose.position.x,
          odom->pose.pose.position.y);
      person_loc = topological_mapper::toGrid(person_loc, map_info_);

      size_t current_graph_id =
        topological_mapper::getClosestIdOnGraphFromEdge(person_loc,
            graph_, current_state.graph_id);

      if (current_graph_id != current_state.graph_id) {
        // A transition has happened. Compute next state and check if robot 
        // needs to be placed.
        size_t next_direction = model_->computeNextDirection(
            current_state.direction, current_state.graph_id, 
            current_graph_id);
        ROS_INFO_STREAM("Transitioning to: " << current_graph_id << 
            ", direction: " << next_direction << 
            ", num_robots: " << robots_at_next_state_);
        current_state_ = model_->canonicalizeState(current_graph_id,
            next_direction, robots_at_next_state_);
        checkRobotPlacementAtCurrentState();
      }
    }

};


int main(int argc, char** argv) {

  ros::init(argc, argv, "vi_robot_positioner");
  boost::shared_ptr<ros::NodeHandle> nh;
  nh.reset(new ros::NodeHandle());
  VIRobotPositioner rp(nh);
  rp.run();

  return 0;
}
