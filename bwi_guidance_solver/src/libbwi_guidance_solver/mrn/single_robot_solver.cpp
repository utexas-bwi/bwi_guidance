#include <fstream>

#include <boost/foreach.hpp>
#include <pluginlib/class_list_macros.h>

#include <bwi_guidance_solver/mrn/single_robot_solver.h>

namespace bwi_guidance_solver {

  namespace mrn {

    std::string SingleRobotSolver::getSolverName() {
      return std::string("Heuristic");
    }

    void SingleRobotSolver::resetSolverSpecific() {
      rng_.reset(new RNG(seed_ * 45 + 2));
    }

    Action SingleRobotSolver::getBestAction(const State& state) {
      // The single robot solver prefers leading if co-located, otherwise actions are taken randomly.
      std::vector<Action> actions;
      model_->getAllActions(state, actions);
      return actions[getBestAction(state, actions, rng_)];
    }

    int SingleRobotSolver::getBestAction(const State& state, 
                                         const std::vector<Action> &actions, 
                                         const boost::shared_ptr<RNG> &rng) {
      /* I believe it is guaranteed that state.graph_id won't be the same as goal_idx prior to this function being
       * called. */
      int lead_graph_idx = model_->shortest_paths_[state.graph_id][goal_idx_][0];
      for (int i = 0; i < actions.size(); ++i) {
        if (actions[i].type == LEAD_PERSON && actions[i].guide_graph_id == lead_graph_idx) {
          return i;
        }
      }

      // Not a co-located state. Return an action randomly.
      return rng->randomInt(actions.size() - 1);
    }

    void SingleRobotSolver::performEpisodeStartComputation(const State &state) {
      if (domain_params_.frame_rate != 0.0f) {
        // MCTS waits for 10 seconds here, but let's wait only for 1 second for now.
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000.0f));
      }
    }

    void SingleRobotSolver::performPostActionComputation(const State &state, float time) {
      if (domain_params_.frame_rate != 0.0f) {
        // MCTS waits for 10 seconds here, but let's wait only for 1 second for now.
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000.0f * time));
      }
    }

  } /* mrn */

} /* bwi_guidance_solver */

PLUGINLIB_EXPORT_CLASS(bwi_guidance_solver::mrn::SingleRobotSolver, bwi_guidance_solver::mrn::Solver)
