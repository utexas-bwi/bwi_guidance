#include <bwi_guidance_solver/mrn/restricted_model.h>

namespace bwi_guidance_solver {

  namespace mrn {
    
    RestrictedModel::RestrictedModel(const bwi_mapper::Graph &graph, 
                                     const nav_msgs::OccupancyGrid &map, 
                                     int goal_idx, 
                                     const MotionModel::Ptr &motion_model,
                                     const HumanDecisionModel::Ptr &human_decision_model,
                                     const TaskGenerationModel::Ptr &task_generation_model,
                                     const Params &params) :
      goal_idx_(goal_idx), 
      params_(params) {

        PersonModel::Params base_model_params;
        base_model_params.frame_rate = params.frame_rate;
        base_model_params.num_robots = params.num_robots;
        base_model_params.avg_robot_speed = params.avg_robot_speed;

        params_.avg_robot_speed /= map.info.resolution;
        avg_human_speed_ = motion_model->getHumanSpeed();

        num_vertices_ = boost::num_vertices(graph);
        computeAdjacentVertices(adjacent_vertices_map_, graph);
        computeShortestPath(shortest_distances_, shortest_paths_, graph);

        /* Kickstart original model */
        base_model_.reset(new PersonModel(graph,
                                          map,
                                          goal_idx,
                                          motion_model,
                                          human_decision_model,
                                          task_generation_model,
                                          base_model_params));
      }

    void RestrictedModel::getActionsAtState(const RestrictedState& state, 
                                            std::vector<RestrictedAction>& actions) {
      int action_counter = 0;

      // If the previous action was DIRECT_HUMAN, then this action is fixed.
      if (state.prev_action.type == DIRECT_PERSON) {
        actions.push_back(RestrictedAction(RELEASE_ROBOT, state.prev_action.robot_id));
        return;
      } 

      // Check if WAIT is a possible action. If LEAD_PERSON has been called, then return WAIT as the only valid action.
      int colocated_robot_id = getColocatedRobotId(state);
      if (state.assist_type != NONE || colocated_robot_id == NONE) {
        actions.push_back(RestrictedAction(WAIT));
        ++action_counter;
      }
      if (state.assist_type == LEAD_PERSON) {
        return;
      }

      // Check if leading a person is allowed here.
      if (colocated_robot_id != NONE) {
        actions.resize(action_counter + adjacent_vertices_map_[state.loc_node].size());
        for (unsigned int adj = 0; adj < adjacent_vertices_map_[state.loc_node].size(); ++adj) {
          actions[action_counter] = 
            RestrictedAction(LEAD_PERSON, colocated_robot_id, adjacent_vertices_map_[state.loc_node][adj]);
          ++action_counter;
        }
      }

      // Check if assigning a robot is allowed here.
      std::vector<int> unassignable_locations;
      for (unsigned int robot = 0; robot < state.robots.size(); ++robot) {
        if (state.robots[robot].help_destination != NONE) {
          unassignable_locations.push_back(state.robots[robot].help_destination);
        }
      }

      if (unassignable_locations.size() < params_.max_assigned_robots) {
        unassignable_locations.insert(unassignable_locations.end(), state.released_locations.begin(), state.released_locations.end());
        for (unsigned int node = 0; node < num_vertices_; ++node) {
          if (std::find(unassignable_locations.begin(), unassignable_locations.end(), node) ==
              unassignable_locations.end()) {
            // Note that the robot id is unknown here.
            actions.push_back(RestrictedAction(ASSIGN_ROBOT, 0, node));
            ++action_counter;
          }
        }
      }

      // Check if releasing a robot is allowed here.
      if (state.prev_action.type == WAIT ||
          state.prev_action.type == RELEASE_ROBOT ||
          state.prev_action.type == DIRECT_PERSON) {
        for (unsigned int robot = 0; robot < state.robots.size(); ++robot) {
          if (state.robots[robot].help_destination != NONE) {
            actions.push_back(RestrictedAction(RELEASE_ROBOT, robot));
          }
        }
      }

      // Finally see if directing a person is allowed here.
      if (colocated_robot_id != NONE && state.prev_action == WAIT) {
        actions.resize(action_counter + adjacent_vertices_map_[state.loc_node].size());
        for (unsigned int adj = 0; adj < adjacent_vertices_map_[state.loc_node].size(); ++adj) {
          actions[action_counter] = 
            RestrictedAction(DIRECT_PERSON, colocated_robot_id, adjacent_vertices_map_[state.loc_node][adj]);
          ++action_counter;
        }
      }

    }

    void RestrictedModel::getFirstAction(const RestrictedState &state, RestrictedAction &action) {
      // This function cannot be optimized without maintaining state. Only present for legacy
      // reasons. Do not call it! For this reason, here is a very suboptimal implementation.
      std::vector<RestrictedAction> actions;
      getActionsAtState(state, actions);
      action = actions[0];
    }

    bool RestrictedModel::getNextAction(const RestrictedState &state, RestrictedAction &action) {
      // This function cannot be optimized without maintaining state. Only present for legacy
      // reasons. Do not call it!
      std::vector<RestrictedAction> actions;
      getActionsAtState(state, actions);
      for (unsigned int action_id = 0; action_id < actions.size() - 1; ++action_id) {
        if (actions[action_id] == action) {
          action = actions[action_id + 1];
          return true;
        }
      }
      return false;
    }

    void RestrictedModel::getAllActions(const RestrictedState &state, std::vector<RestrictedAction>& actions) {
      getActionsAtState(state, actions);
    }

    void RestrictedModel::takeAction(const RestrictedState &state, 
                                     const RestrictedAction &action, 
                                     float &reward, 
                                     RestrictedState &next_state, 
                                     bool &terminal, 
                                     int &depth_count, 
                                     boost::shared_ptr<RNG> rng) {

      Action mapped_action = action;
      if (action.type == ASSIGN_ROBOT) {
        mapped_action.robot_id = selectBestRobotForTask(state, 
                                                        action.node,
                                                        avg_human_speed_,
                                                        params_.avg_robot_speed,
                                                        shortest_distances_);
      }

      base_model_->takeAction(state, 
                              mapped_action, 
                              reward, 
                              next_state, 
                              terminal, 
                              depth_count, 
                              rng);

      next_state.prev_action = action;
      if (action.type == WAIT) {
        next_state.released_locations.clear();
      } else if (action.type == RELEASE_ROBOT) {
        next_state.released_locations.push_back(state.robots[action.robot_id].help_destination);
      } else {
        next_state.released_locations = state.released_locations;
      }
    }

    int RestrictedModel::getColocatedRobotId(const RestrictedState& state) {
      // Figure out if there is a robot at the current position
      for (int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {
        if ((state.robots[robot_id].help_destination == state.loc_node) &&
            isRobotExactlyAt(state.robots[robot_id], state.loc_node)) {
          return robot_id;
        }
      }
      return NONE;
    }

  } /* mrn */

} /* bwi_guidance */
