#include <fstream>

#include <boost/foreach.hpp>
#include <pluginlib/class_list_macros.h>

#include <bwi_guidance_solver/mrn/heuristic_solver.h>

namespace bwi_guidance_solver {

  namespace mrn {

    bool HeuristicSolver::initializeSolverSpecific(Json::Value &params) {
      // Since the heuristic solver reuses the IRM heuristic solver, we need to call the initialize function
      // for that solver here.
      irm::Domain::Params irm_domain_params; // Get a copy of the default parameters.
      return irm_hs_.initialize(irm_domain_params, params, map_, graph_, base_directory_);
    }

    void HeuristicSolver::resetSolverSpecific() {
      // Since the heuristic solver reuses the IRM heuristic solver, we need to call the reset function
      // for that solver here. Since we don't really need to supply a model, simply provide an empty boost shared ptr.
      boost::shared_ptr<irm::PersonModel> uninitialized_irm_model;
      irm_hs_.reset(uninitialized_irm_model, seed_, goal_idx_);
    }

    std::string HeuristicSolver::getSolverName() {
      return std::string("Heuristic");
    }

    Action HeuristicSolver::getBestAction(const State& state) {

      // Map the current state into that of the QRR Heuristic Solver, and 
      // use that solver out of the box. Then map the action to the new domain
      // and return the mapped action

      irm::State mapped_state;
      mapped_state.graph_id = state.graph_id;
      mapped_state.direction = state.direction;
      bool cur_robot_available = Solver::model_->isAssignedRobotColocated(state);
      mapped_state.robot_direction = (cur_robot_available) ? DIR_UNASSIGNED : NONE;
      mapped_state.num_robots_left = 5;
      mapped_state.visible_robot = NONE;

      // Figure out the vertices that cannot be reached in time
      boost::shared_ptr<std::vector<int> > blacklisted_vertices;
      blacklisted_vertices.reset(new std::vector<int>);
      blacklisted_vertices->insert(blacklisted_vertices->end(), 
                                   state.relieved_locations.begin(), state.relieved_locations.end());
      if (params_.improved) {
        BOOST_FOREACH(const int& vtx, irm_hs_.visible_vertices_map_[mapped_state.graph_id]) {
          if (state.in_use_robots.size() != 0) {
            if (vtx == state.in_use_robots[0].destination) {
              continue;
            }
          }
          bool reach_in_time;
          float time_to_destination = 
            bwi_mapper::getShortestPathDistance(state.graph_id, vtx, Solver::graph_) / 
            (domain_params_.human_speed / map_.info.resolution);
          Solver::model_->selectBestRobotForTask(state, vtx, time_to_destination, reach_in_time);
          if (!reach_in_time) {
            blacklisted_vertices->push_back(vtx);
          }
        }
      }

      // Get the best action 
      irm::Action action = irm_hs_.getBestActionWithBlacklistedVertices(mapped_state, blacklisted_vertices);
      Action mapped_action;
      switch(action.type) {
        case irm::DO_NOTHING:
          if (state.in_use_robots.size() != 0) {
            // The approach does not require a robot. Release acquired robot.
            mapped_action = Action(RELEASE_ROBOT, state.in_use_robots[0].destination, NONE);
          }
          return mapped_action;
        case irm::DIRECT_PERSON:
          mapped_action = Action(GUIDE_PERSON, mapped_state.graph_id, action.graph_id);
          return mapped_action;
        case irm::PLACE_ROBOT:
          if (state.in_use_robots.size() != 0) {
            if (state.in_use_robots[0].destination != action.graph_id) {
              // The approach wants to place a robot at a diff location, release the current one
              mapped_action = Action(RELEASE_ROBOT, state.in_use_robots[0].destination, NONE);
            } else {
              // A robot is already heading there. Carry on, nothing to do here.
            }
          } else {
            mapped_action = Action(ASSIGN_ROBOT, action.graph_id, DIR_UNASSIGNED);
          }
          return mapped_action;
      }
    }

    void HeuristicSolver::performEpisodeStartComputation(const State &state) {
      if (domain_params_.frame_rate != 0.0f) {
        // MCTS waits for 10 seconds here, but let's wait only for 1 second for now.
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000.0f));
      }
    }

    void HeuristicSolver::performPostActionComputation(const State &state, float time) {
      if (domain_params_.frame_rate != 0.0f) {
        // MCTS waits for 10 seconds here, but let's wait only for 1 second for now.
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000.0f * time));
      }
    }

  } /* mrn */

} /* bwi_guidance_solver */

PLUGINLIB_EXPORT_CLASS(bwi_guidance_solver::mrn::HeuristicSolver, bwi_guidance_solver::mrn::Solver)
