#include <bwi_guidance_solver/irm/solver.h>

namespace bwi_guidance_solver {

  namespace irm {

    bool Solver::initialize(Domain::Params &domain_params, 
                            Json::Value &params, 
                            const nav_msgs::OccupancyGrid &map,
                            const bwi_mapper::Graph &graph, 
                            const std::string &base_directory) {
      domain_params_ = domain_params;
      general_params_.fromJson(params);
      map_ = map;
      graph_ = graph;

      // Compute the base directory and create it.
      std::ostringstream parametrized_dir_ss;
      parametrized_dir_ss << std::fixed << std::setprecision(2);
      parametrized_dir_ss << base_directory << "/ros" << general_params_.reward_on_success << 
        "-rs" << general_params_.reward_structure;
      base_directory_ = parametrized_dir_ss.str();
      if (!boost::filesystem::is_directory(base_directory_) &&
          !boost::filesystem::create_directory(base_directory_))
      {
        return false;
      }

      return this->initializeSolverSpecific(params);
    }

    void Solver::reset(const boost::shared_ptr<PersonModel> &model, int seed, int goal_idx) {
      seed_ = seed;
      model_ = model;
      goal_idx_ = goal_idx;
      this->resetSolverSpecific();
    }

    std::map<std::string, std::string> Solver::getParamsAsMap() { 
      std::map<std::string, std::string> stringMap = this->getParamsAsMapSolverSpecific();
      std::map<std::string, std::string> general_params_map = general_params_.asMap();
      stringMap.insert(general_params_map.begin(), general_params_map.end());
      return stringMap;
    }

    bool Solver::initializeSolverSpecific(Json::Value &params) { return true; }
    void Solver::resetSolverSpecific() {}
    void Solver::precomputeAndSavePolicy(int problem_identifier) {}
    void Solver::performEpisodeStartComputation(const State &state) {}
    void Solver::performPostActionComputation(const State &state, float distance) {}
    std::map<std::string, std::string> Solver::getParamsAsMapSolverSpecific() { 
      return std::map<std::string, std::string>();
    }
  } /* irm - InstantaneousRobotMotion */

} /* bwi_guidance_solver */

