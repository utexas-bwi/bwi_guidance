#include <fstream>
#include <pluginlib/class_list_macros.h>

#include <bwi_guidance_solver/mrn/mcts_solver.h>
/* #include <bwi_guidance_solver/mrn/abstract_mapping.h> */
#include <bwi_guidance_solver/mrn/single_robot_solver.h>

#include <bwi_rl/planning/IdentityStateMapping.h>
#include <bwi_rl/planning/ModelUpdaterSingle.h>
#include <bwi_rl/planning/RandomPolicy.h>

namespace bwi_guidance_solver {

  namespace mrn {

    bool MCTSSolver::initializeSolverSpecific(Json::Value &params) {
      mcts_solver_params_.fromJson(params);
      mcts_params_.fromJson(params);
      extended_mdp_params_.fromJson(params);
      return true;
    }

    void MCTSSolver::resetSolverSpecific() {
      // Create the RNG required for mcts rollouts
      
      // Set the MDP parameters and initialize the MDP.
      boost::shared_ptr<RestrictedModel> extended_model(new RestrictedModel(graph_, 
                                                                            map_, 
                                                                            goal_idx_, 
                                                                            motion_model_,
                                                                            human_decision_model_,
                                                                            task_generation_model_,
                                                                            extended_mdp_params_));

      boost::shared_ptr<RNG> mcts_rng(new RNG(1 * (seed_ + 1)));
      boost::shared_ptr<ModelUpdaterSingle<ExtendedState, Action> >
        mcts_model_updator(new ModelUpdaterSingle<ExtendedState, Action>(extended_model));
      boost::shared_ptr<StateMapping<ExtendedState> > mcts_state_mapping;
      // if (mcts_solver_params_.use_abstract_mapping) {
      //   mcts_state_mapping.reset(new AbstractMapping);
      // } else {
        mcts_state_mapping.reset(new IdentityStateMapping<ExtendedState>);
      /* } */
      
      boost::shared_ptr<SingleRobotSolver> default_policy;
      default_policy.reset(new SingleRobotSolver);

      // TODO fill this from the current solver values.
      Json::Value empty_json; // Force the single robot solver to use default parameters.
      default_policy->initialize(domain_params_, empty_json, map_, graph_, robot_home_base_, base_directory_);
      default_policy->reset(seed_, goal_idx_);

      mcts_.reset(new MultiThreadedMCTS<ExtendedState, ExtendedStateHash, Action>(default_policy,
                                                                                  mcts_model_updator, 
                                                                                  mcts_state_mapping, 
                                                                                  mcts_rng,
                                                                                  mcts_params_));
    }

    Action MCTSSolver::getBestAction(const ExtendedState &state) {
      return mcts_->selectWorldAction(state);
    }

    void MCTSSolver::performEpisodeStartComputation(const ExtendedState &state) {
      float time = mcts_solver_params_.initial_planning_time * mcts_solver_params_.planning_time_multiplier;
      search(state, time);
    }

    std::string MCTSSolver::getSolverName() {
      return std::string("UCT");
    }

    void MCTSSolver::performPostActionComputation(const ExtendedState &state, float distance) {
      // NOTE This assumes the human is moving at a speed of 1m/s.
      float time = distance * mcts_solver_params_.planning_time_multiplier;
      search(state, time);
    }

    void MCTSSolver::search(const ExtendedState &state, float time) {
      // Only perform the search if the search time is greater than 0, and there is something to search over.
      if (time > 0.0f) {
        unsigned int unused_rollout_termination_count;
        mcts_->search(state, unused_rollout_termination_count, time);
      }
    }

    std::map<std::string, std::string> MCTSSolver::getParamsAsMapSolverSpecific() {
      std::map<std::string, std::string> mcts_solver_params = mcts_solver_params_.asMap();
      std::map<std::string, std::string> mcts_params = mcts_params_.asMap();
      mcts_solver_params.insert(mcts_params.begin(), mcts_params.end());
      return mcts_solver_params;
    }

  }

}

PLUGINLIB_EXPORT_CLASS(bwi_guidance_solver::mrn::MCTSSolver, bwi_guidance_solver::mrn::Solver)
