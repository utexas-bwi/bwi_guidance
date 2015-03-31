#include <fstream>

#include <boost/foreach.hpp>
#include <pluginlib/class_list_macros.h>

#include <bwi_guidance_solver/mrn/single_robot_solver.h>

namespace bwi_guidance_solver {

  namespace mrn {

    std::string SingleRobotSolver::getSolverName() {
      return std::string("SingleRobot");
    }

    bool SingleRobotSolver::initializeSolverSpecific(Json::Value &params) {
      computeShortestPath(shortest_distances_, shortest_paths_, graph_);
      computeAdjacentVertices(adjacent_vertices_map_, graph_);
      return true;
    }

    void SingleRobotSolver::resetSolverSpecific() {
      rng_.reset(new RNG(seed_ * 45 + 2));
    }

    Action SingleRobotSolver::getBestAction(const ExtendedState& state) {
      // The single robot solver prefers leading if co-located, otherwise actions are taken randomly.
      std::vector<Action> actions;
      model_->getAllActions(state, actions);
      return actions[getBestAction(state, actions, rng_)];
    }

    int SingleRobotSolver::getBestAction(const ExtendedState& state, 
                                         const std::vector<Action> &actions, 
                                         const boost::shared_ptr<RNG> &rng) {

      // Often, there will only be 1 valid action. Simply choose it without applying any other logic.
      if (actions.size() == 1) {
        return 0;
      }

      int colocated_robot_id = getColocatedRobotId(state);
      if (colocated_robot_id == NONE) {

        // Worry about assigning and releasing robots.
        std::vector<int> states;
        int current_id = state.loc_node;
        float current_direction = bwi_mapper::getNodeAngle(state.loc_prev, state.loc_node, graph_);
        if (state.assist_type != NONE) {
          current_direction = bwi_mapper::getNodeAngle(state.loc_node, state.assist_loc, graph_);
        }
        while(true) {

          states.push_back(current_id);
          bwi_mapper::Point2f loc = bwi_mapper::getLocationFromGraphId(current_id, graph_);

          // Compute all adjacent vertices from this location
          std::vector<int> &adjacent_vertices = adjacent_vertices_map_[current_id];
          std::vector<int> &visible_vertices = adjacent_vertices_map_[current_id];

          // Check vertex that has most likely transition
          int next_vertex = -1;
          float next_vertex_closeness = M_PI / 4;
          float next_angle = 0.0f;
          for (std::vector<int>::const_iterator av = adjacent_vertices.begin(); av != adjacent_vertices.end(); ++av) {
            bwi_mapper::Point2f next_loc = bwi_mapper::getLocationFromGraphId(*av, graph_);
            bwi_mapper::Point2f diff_loc = next_loc - loc;
            float angle = atan2f(diff_loc.y, diff_loc.x);
            // Wrap angle around direction
            while (angle <= current_direction - M_PI) angle += 2 * M_PI;
            while (angle > current_direction + M_PI) angle -= 2 * M_PI;
            // Take difference
            float closeness = fabs(angle - current_direction);
            if (closeness < next_vertex_closeness) {
              next_vertex = *av;
              next_vertex_closeness = closeness;
              next_angle = angle; 
            }
          }

          if (next_vertex == -1) {
            // No close vertices found, break
            break;
          }

          bool next_visible = 
            std::find(visible_vertices.begin(), visible_vertices.end(), next_vertex) != visible_vertices.end();

          if (!next_visible) {
            // The most probable location in path is no longer visible, hence no longer
            // probable
            break;
          }

          // get normalize direction and id
          current_direction = atan2f(sinf(next_angle), cosf(next_angle)); 
          current_id = next_vertex;
        }

        // If a currently assigned robot is not in the above states, then release it if possible. 
        bool assigned_robot_not_released = false;
        for (int i = 0; i < state.robots.size(); ++i) {
          if (state.robots[i].help_destination != NONE) {
            if (std::find(states.begin(), states.end(), state.robots[i].help_destination) == states.end()) {
              // This robot is no longer in the expected path of the person. See if we can release this robot.
              for (int i = 0; i < actions.size(); ++i) {
                if (actions[i].type == RELEASE_ROBOT && actions[i].robot_id == i) {
                  return i;
                }
              }
            }
            assigned_robot_not_released = true;
          }
        }

        // If any assigned robot exists that we did not release. Let's play this out. Return wait.
        if (assigned_robot_not_released) {
          for (int i = 0; i < actions.size(); ++i) {
            if (actions[i].type == WAIT) {
              return i;
            }
          }
        }

        // OK. so states contains everywhere we think the human is gonna go. See if we can send a robot to one of 
        // these locations.
        for (int i = 0; i < states.size(); ++i) {
          bool reach_in_time = false;
          int best_robot_id = selectBestRobotForTask(state, 
                                                     states[i], 
                                                     domain_params_.human_speed / map_.info.resolution, 
                                                     domain_params_.robot_speed / map_.info.resolution, 
                                                     shortest_distances_, 
                                                     reach_in_time); 
          if (reach_in_time) {
            for (int j = 0; j < actions.size(); ++j) {
              if (actions[j].type == ASSIGN_ROBOT && actions[j].node == states[i]) {
                // Robot_id can be -1 in the case of restricted action space.
                if (actions[j].robot_id == -1 || actions[j].robot_id == best_robot_id) {
                  return j;
                } 
              }
            }
          }
        }

      } else {

        // If the colocated robot id has not helped the person, then lead the person to the goal.
        if (state.assist_type == NONE) {
          int lead_graph_idx = shortest_paths_[state.loc_node][goal_idx_][0];
          for (int i = 0; i < actions.size(); ++i) {
            if ((actions[i].type == LEAD_PERSON) && 
                (actions[i].robot_id == colocated_robot_id) && 
                (actions[i].node == lead_graph_idx)) {
              return i;
            }
          }
        }

      }

      // Could not assign a robot in time. Wait and do nothing.
      for (int i = 0; i < actions.size(); ++i) {
        if (actions[i].type == WAIT) {
          return i;
        }
      }

      // Somehow if wait is not available (should not happen), return a random valid action.
      return rng->randomInt(actions.size() - 1);
    }

    void SingleRobotSolver::performEpisodeStartComputation(const ExtendedState &state) {
      if (domain_params_.frame_rate != 0.0f) {
        // MCTS waits for 10 seconds here, but let's wait only for 1 second for now.
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000.0f));
      }
    }

    void SingleRobotSolver::performPostActionComputation(const ExtendedState &state, float time, bool new_action) {
      if (domain_params_.frame_rate != 0.0f) {
        // MCTS waits for 10 seconds here, but let's wait only for 1 second for now.
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000.0f * time));
      }
    }

  } /* mrn */

} /* bwi_guidance_solver */

PLUGINLIB_EXPORT_CLASS(bwi_guidance_solver::mrn::SingleRobotSolver, bwi_guidance_solver::mrn::Solver)
