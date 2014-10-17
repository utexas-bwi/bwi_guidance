#include <fstream>

#include <boost/foreach.hpp>
#include <pluginlib/class_list_macros.h>

#include <bwi_guidance_solver/mrn/single_robot_solver.h>

namespace bwi_guidance_solver {

  namespace mrn {

    std::string SingleRobotSolver::getSolverName() {
      return std::string("Heuristic");
    }

    Action SingleRobotSolver::getBestAction(const State& state) {
      // TODO should at least handle the case if robot is not co-located.
      std::vector<size_t> path_from_goal;
      bwi_mapper::getShortestPathWithDistance(state.graph_id, goal_idx_, path_from_goal, graph_);
      path_from_goal.pop_back();
      if (path_from_goal.size() == 0) {
        return Action(LEAD_PERSON, state.graph_id, goal_idx_);
      } else {
        return Action(LEAD_PERSON, state.graph_id, path_from_goal.back());
      }
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
