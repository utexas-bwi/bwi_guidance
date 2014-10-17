#include <bwi_guidance_solver/mrn/solver.h>

namespace bwi_guidance_solver {

  namespace mrn {

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
      parametrized_dir_ss << base_directory << "/mr" << general_params_.max_robots_in_use << 
        "-avvd" << general_params_.action_vertex_visiblity_depth <<
        "-avad" << general_params_.action_vertex_adjacency_depth << 
        "-vr" << general_params_.visibility_range;
      base_directory_ = parametrized_dir_ss.str();
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

      model_.reset(new PersonModel(graph_, 
                                   map_, 
                                   goal_idx_, 
                                   domain_params_.frame_rate,
                                   general_params_.max_robots_in_use,
                                   general_params_.action_vertex_visiblity_depth,
                                   general_params_.action_vertex_adjacency_depth,
                                   general_params_.visibility_range,
                                   domain_params_.human_speed,
                                   domain_params_.robot_speed,
                                   domain_params_.utility_multiplier,
                                   domain_params_.use_shaping_reward,
                                   domain_params_.discourage_bad_assignments));

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
    void Solver::performPostActionComputation(const State &state, float time) {}
    std::map<std::string, std::string> Solver::getParamsAsMapSolverSpecific() { 
      return std::map<std::string, std::string>();
    }
  } /* irm - InstantaneousRobotMotion */

} /* bwi_guidance_solver */

