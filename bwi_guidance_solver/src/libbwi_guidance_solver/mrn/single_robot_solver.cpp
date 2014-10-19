#include <fstream>

#include <boost/foreach.hpp>
#include <pluginlib/class_list_macros.h>

#include <bwi_guidance_solver/mrn/single_robot_solver.h>

namespace bwi_guidance_solver {

  namespace mrn {

    std::string SingleRobotSolver::getSolverName() {
      return std::string("SingleRobot");
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

      // Not a co-located state. Return an action randomly. Note that this will never happen if this policy is used
      // from start to end.
       
      // return rng->randomInt(actions.size() - 1);
      
      // Instead of returning a random action, let's try something fancy here.
      std::vector<int> states;
      int current_id = state.graph_id;
      float current_direction = getAngleInRadians(state.direction);
      while(true) {

        states.push_back(current_id);
        bwi_mapper::Point2f loc = bwi_mapper::getLocationFromGraphId(current_id, graph_);

        // Compute all adjacent vertices from this location
        std::vector<int> &adjacent_vertices = model_->adjacent_vertices_map_[current_id];
        std::vector<int> &visible_vertices = model_->adjacent_vertices_map_[current_id];

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

      // If assigned robot is not in states, then let it go!
      for (int i = 0; i < state.in_use_robots.size(); ++i) {
        if (std::find(states.begin(), states.end(), state.in_use_robots[i].destination) == states.end()) {
          // This robot is no longer in the expected path of the person. See if we can release this robot.
          for (int i = 0; i < actions.size(); ++i) {
            if (actions[i].type == RELEASE_ROBOT && actions[i].at_graph_id == state.in_use_robots[i].destination) {
              return i;
            }
          }
        }
      }

      if (state.in_use_robots.size() != 0) {
        // This means that there is an assigned robot that we did not release. Let's play this out. Return wait.
        for (int i = 0; i < actions.size(); ++i) {
          if (actions[i].type == WAIT) {
            return i;
          }
        }
      }

      // OK. so states contains everywhere we think the human is gonna go.
      // Select the first vtx to which a robot can reach in time and assign a robot there.
      for (int i = 0; i < states.size(); ++i) {
        int retval = -1;
        for (int j = 0; j < actions.size(); ++j) {
          if (actions[j].type == ASSIGN_ROBOT && actions[j].at_graph_id == states[i]) {
            retval = j;
            break;
          }
        }
        if (retval == -1) { // We cannot assign a robot here.
          continue;
        }

        // float time_to_destination = model_->shortest_distances_[state.graph_id][states[i]] /
        //   (domain_params_.human_speed / map_.info.resolution);
        // bool reach_in_time;
        // Solver::model_->selectBestRobotForTask(state, states[i], time_to_destination, reach_in_time);
        // if (reach_in_time) {
          return retval;
        /* } */
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
