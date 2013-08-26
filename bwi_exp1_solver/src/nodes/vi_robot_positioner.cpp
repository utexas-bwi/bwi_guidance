#include<fstream>

#include <bwi_exp1_solver/ValueIteration.h>
#include <bwi_exp1_solver/person_estimator.h>
#include <bwi_exp1_solver/person_model.h>
#include <topological_mapper/map_loader.h>
#include <bwi_exp1/base_robot_positioner.h>

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

    state_t current_state_;
    unsigned int robots_at_next_state_;

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

      const Instance& instance = 
        getInstance(experiment_, instance_name);

      topological_mapper::Point2f start_point(instance.start_loc.x,
          instance.start_loc.y);
      start_point = topological_mapper::toGrid(start_point, map_info_);

      size_t prev_graph_id = 
        topological_mapper::getClosestIdOnGraph(start_point, graph_);
      size_t direction = 
        model_->getDirectionFromAngle(instance.start_loc.yaw);

      current_state_ = model_->canonicalizeState(prev_graph_id, 
          direction, num_robots_available_);
      checkRobotPlacementAtCurrentState();

    }

    virtual void checkRobotPlacementAtCurrentState() {
      boost::mutex::scoped_lock lock(robot_modification_mutex_);

      // First check if we need to place a robot according to VI policy
      State previous_state = model_->resolveState(current_state_);

      action_t action_idx = vi_->getBestAction(current_state_);
      Action action = model_->resolveAction(current_state_, action_idx);
      if (action.type == PLACE_ROBOT) {
        std::cout << "FOUND ROBOT. Robot points towards " << action.graph_id << std::endl;
      }

      robots_at_next_state_ = previous_state.num_robots_left - 1;
    }

    virtual void odometryCallback(const nav_msgs::Odometry::ConstPtr odom) {

      State previous_state = model_->resolveState(current_state_);

      topological_mapper::Point2f person_loc(
          odom->pose.pose.position.x,
          odom->pose.pose.position.y);
      person_loc = topological_mapper::toGrid(person_loc, map_info_);

      size_t current_graph_id =
        topological_mapper::getClosestIdOnGraphFromEdge(person_loc,
            graph_, previous_state.graph_id);

      if (current_graph_id != previous_state.graph_id) {
        // A transition has happened. Compute next state and check if robot 
        // needs to be placed.
        size_t next_direction = model_->computeNextDirection(
            previous_state.direction, previous_state.graph_id, 
            current_graph_id);
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
