#include <bwi_guidance_solver/mrn/solver.h>

namespace bwi_guidance_solver {

  namespace mrn {

    bool Solver::initialize(Domain::Params &domain_params,
                            Json::Value &params,
                            const nav_msgs::OccupancyGrid &map,
                            const bwi_mapper::Graph &graph,
                            const std::vector<int> &robot_home_base,
                            const std::string &base_directory) {
      domain_params_ = domain_params;
      map_ = map;
      graph_ = graph;
      robot_home_base_ = robot_home_base;

      // Compute the base directory and create it.
      base_directory_ = base_directory;
      if (!boost::filesystem::is_directory(base_directory_) &&
          !boost::filesystem::create_directory(base_directory_))
      {
        return false;
      }

      return this->initializeSolverSpecific(params);
    }

    void Solver::reset(int seed, int goal_idx) {
      seed_ = seed;
      goal_idx_ = goal_idx;

      // Initialize the transition model.
      motion_model_.reset(new MotionModel(graph_,
                                          domain_params_.robot_speed / map_.info.resolution,
                                          domain_params_.human_speed / map_.info.resolution));

      task_generation_model_.reset(new TaskGenerationModel(robot_home_base_,
                                                           graph_,
                                                           domain_params_.utility_multiplier));

      human_decision_model_.reset(new HumanDecisionModel(graph_));

      // Set the MDP parameters and initialize the MDP.
      PersonModel::Params params;
      params.frame_rate = 0.0f; // This version of the model should never visualize, as it is used for sampling only.

      model_.reset(new PersonModel(graph_,
                                   map_,
                                   goal_idx_,
                                   motion_model_,
                                   human_decision_model_,
                                   task_generation_model_,
                                   params));


      this->resetSolverSpecific();
    }

    std::map<std::string, std::string> Solver::getParamsAsMap() {
      return this->getParamsAsMapSolverSpecific();
    }

    bool Solver::initializeSolverSpecific(Json::Value &params) { return true; }
    void Solver::resetSolverSpecific() {}
    void Solver::precomputeAndSavePolicy(int problem_identifier) {}
    void Solver::performEpisodeStartComputation(const ExtendedState &state) {}
    void Solver::performPostActionComputation(const ExtendedState &state, float time, bool new_action) {}
    std::map<std::string, std::string> Solver::getParamsAsMapSolverSpecific() {
      return std::map<std::string, std::string>();
    }
  } /* irm - InstantaneousRobotMotion */

} /* bwi_guidance_solver */

