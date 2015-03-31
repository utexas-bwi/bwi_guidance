#include <fstream>
#include <pluginlib/class_list_macros.h>

#include <bwi_guidance_solver/irm/mcts_solver.h>
#include <bwi_rl/planning/IdentityStateMapping.h>
#include <bwi_rl/planning/ModelUpdaterSingle.h>
#include <bwi_rl/planning/RandomPolicy.h>

namespace bwi_guidance_solver {

  namespace irm {

    bool MCTSSolver::initializeSolverSpecific(Json::Value &params) {
      mcts_solver_params_.fromJson(params);
      mcts_params_.fromJson(params);
      mcts_params_.rewardBound /= map_.info.resolution;
      return true;
    }

    void MCTSSolver::resetSolverSpecific() {
      // Create the RNG required for mcts rollouts
      boost::shared_ptr<RNG> mcts_rng(new RNG(1 * (seed_ + 1)));
      boost::shared_ptr<ModelUpdaterSingle<State, Action> >
        mcts_model_updator(new ModelUpdaterSingle<State, Action>(model_));
      boost::shared_ptr<IdentityStateMapping<State> > mcts_state_mapping(new IdentityStateMapping<State>);
      boost::shared_ptr<RandomPolicy<State, Action> > random_default_policy(new RandomPolicy<State, Action>);
      mcts_.reset(new MultiThreadedMCTS<State, StateHash, Action>(random_default_policy,
                                                                  mcts_model_updator, 
                                                                  mcts_state_mapping, 
                                                                  mcts_rng,
                                                                  mcts_params_));
    }

    Action MCTSSolver::getBestAction(const State &state) {
      return mcts_->selectWorldAction(state);
    }

    void MCTSSolver::performEpisodeStartComputation(const State &state) {
      float time = mcts_solver_params_.initial_planning_time * mcts_solver_params_.planning_time_multiplier;
      search(state, time);
    }

    std::string MCTSSolver::getSolverName() {
      return std::string("UCT");
    }

    void MCTSSolver::performPostActionComputation(const State &state, float distance) {
      // NOTE This assumes the human is moving at a speed of 1m/s.
      float time = distance * mcts_solver_params_.planning_time_multiplier;
      search(state, time);
    }

    void MCTSSolver::search(const State &state, float time) {
      // Only perform the search if the search time is greater than 0, and there is something to search over.
      if ((time > 0.0f) && (state.num_robots_left != 0 || state.visible_robot != NONE)) {
        unsigned int unused_rollout_termination_count;
        mcts_->search(state, unused_rollout_termination_count, time);
      }
    }

    std::map<std::string, std::string> MCTSSolver::getParamsAsMapSolverSpecific() {
      std::map<std::string, std::string> mcts_solver_params = mcts_solver_params_.asMap();
      std::map<std::string, std::string> mcts_params = mcts_params_.asMap();
      mcts_params["rewardBound"] = 
        boost::lexical_cast<std::string>(boost::lexical_cast<float>(mcts_params["rewardBound"]) * map_.info.resolution);
      mcts_solver_params.insert(mcts_params.begin(), mcts_params.end());
      return mcts_solver_params;
    }

  }

}

PLUGINLIB_EXPORT_CLASS(bwi_guidance_solver::irm::MCTSSolver, bwi_guidance_solver::irm::Solver)
