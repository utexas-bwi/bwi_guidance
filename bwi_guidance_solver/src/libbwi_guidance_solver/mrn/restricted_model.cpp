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

        avg_robot_speed_ = motion_model->getRobotSpeed();
        avg_human_speed_ = motion_model->getHumanSpeed();

        num_vertices_ = boost::num_vertices(graph);
        computeAdjacentVertices(adjacent_vertices_map_, graph);
        computeShortestPath(shortest_distances_, shortest_paths_, graph);
        computeVisibleVertices(visible_vertices_map_, graph, map, 0.0f);
        action_vertices_map_ = visible_vertices_map_;

        /* Kickstart original model */
        base_model_.reset(new PersonModel(graph,
                                          map,
                                          goal_idx,
                                          motion_model,
                                          human_decision_model,
                                          task_generation_model,
                                          base_model_params));
      }

    void RestrictedModel::getActionsAtState(const ExtendedState& state, 
                                            std::vector<Action>& actions) {
      actions.clear();
      int action_counter = 0;

      // If the previous action was DIRECT_HUMAN, then this action is fixed.
      if (state.prev_action.type == DIRECT_PERSON) {
        actions.push_back(Action(RELEASE_ROBOT, state.prev_action.robot_id));
        return;
      } 

      // Check if WAIT is a possible action. If LEAD_PERSON has been called, then return WAIT as the only valid action.
      int colocated_robot_id = getColocatedRobotId(state);
      if (state.assist_type != NONE || colocated_robot_id == NONE) {
        actions.push_back(Action(WAIT));
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
            Action(LEAD_PERSON, colocated_robot_id, adjacent_vertices_map_[state.loc_node][adj]);
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
        for (unsigned int adj = 0; adj < action_vertices_map_[state.loc_node].size(); ++adj) {
          int node = action_vertices_map_[state.loc_node][adj];
          if (std::find(unassignable_locations.begin(), unassignable_locations.end(), node) ==
              unassignable_locations.end()) {
            // Note that the robot id is unknown here, so we'll use -1.
            actions.push_back(Action(ASSIGN_ROBOT, -1, node));
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
            actions.push_back(Action(RELEASE_ROBOT, robot));
          }
        }
      }

      // Finally see if directing a person is allowed here.
      if (colocated_robot_id != NONE && state.prev_action == WAIT) {
        actions.resize(action_counter + adjacent_vertices_map_[state.loc_node].size());
        for (unsigned int adj = 0; adj < adjacent_vertices_map_[state.loc_node].size(); ++adj) {
          actions[action_counter] = 
            Action(DIRECT_PERSON, colocated_robot_id, adjacent_vertices_map_[state.loc_node][adj]);
          ++action_counter;
        }
      }

    }

    void RestrictedModel::getFirstAction(const ExtendedState &state, Action &action) {
      // This function cannot be optimized without maintaining state. Only present for legacy
      // reasons. Do not call it! For this reason, here is a very suboptimal implementation.
      std::vector<Action> actions;
      getActionsAtState(state, actions);
      action = actions[0];
    }

    bool RestrictedModel::getNextAction(const ExtendedState &state, Action &action) {
      // This function cannot be optimized without maintaining state. Only present for legacy
      // reasons. Do not call it!
      std::vector<Action> actions;
      getActionsAtState(state, actions);
      for (unsigned int action_id = 0; action_id < actions.size() - 1; ++action_id) {
        if (actions[action_id] == action) {
          action = actions[action_id + 1];
          return true;
        }
      }
      return false;
    }

    void RestrictedModel::getAllActions(const ExtendedState &state, std::vector<Action>& actions) {
      getActionsAtState(state, actions);
    }

    void RestrictedModel::takeAction(const ExtendedState &state, 
                                 const Action &action, 
                                 float &reward, 
                                 ExtendedState &next_state, 
                                 bool &terminal, 
                                 int &depth_count, 
                                 boost::shared_ptr<RNG> rng) {

      float unused_utility_loss, unused_time_loss;
      std::vector<State> unused_frame_vector;
      takeAction(state, 
                 action, 
                 reward, 
                 next_state, 
                 terminal, 
                 depth_count, 
                 rng,
                 unused_time_loss, 
                 unused_utility_loss, 
                 unused_frame_vector);
    }

    void RestrictedModel::takeAction(const ExtendedState &state, 
                                     const Action &action, 
                                     float &reward, 
                                     ExtendedState &next_state, 
                                     bool &terminal, 
                                     int &depth_count,
                                     boost::shared_ptr<RNG> &rng,
                                     float &time_loss,
                                     float &utility_loss,
                                     std::vector<State> &frame_vector) {


      Action mapped_action = action;
      if (mapped_action.type == ASSIGN_ROBOT && mapped_action.robot_id == -1) {
        bool unused_reach_in_time;
        mapped_action.robot_id = selectBestRobotForTask(state, 
                                                        action.node,
                                                        avg_human_speed_,
                                                        avg_robot_speed_,
                                                        shortest_distances_,
                                                        unused_reach_in_time);
      }

      base_model_->takeAction(state, 
                              mapped_action, 
                              reward, 
                              next_state, 
                              terminal, 
                              depth_count, 
                              rng,
                              time_loss,
                              utility_loss,
                              frame_vector);

      // next_state.robot_provided_help = state.robot_provided_help;
      // if (action.type == RELEASE_ROBOT) {
      //   std::vector<int>::iterator it = 
      //     std::find(next_state.robot_provided_help.begin(), next_state.robot_provided_help.end(), action.robot_id);
      //   if (it == next_state.robot_provided_help.end()) {
      //     reward -= 2.0f;
      //   } else {
      //     next_state.robot_provided_help.erase(it);
      //   }
      // } 

      // if (action.type == LEAD_PERSON || action.type == DIRECT_PERSON) {
      //   next_state.robot_provided_help.push_back(action.robot_id);
      // }
      
      // float max_robot_utility = 0.0f;
      // for (int r = 0; r < state.robots.size(); ++r) {
      //   if (state.robots[r].help_destination != NONE && state.robots[r].tau_u > max_robot_utility) {
      //     max_robot_utility = state.robots[r].tau_u;
      //   }
      // }

      // if (max_robot_utility > 0.0f && utility_loss < 1e-3 && action.type == WAIT && state.assist_type != LEAD_PERSON) {
      //   reward -= 1000.0f;
      // }

      next_state.prev_action = action;
      if (action.type == WAIT) {
        next_state.released_locations.clear();
      } else if (action.type == RELEASE_ROBOT) {
        next_state.released_locations.push_back(state.robots[action.robot_id].help_destination);
      } else {
        next_state.released_locations = state.released_locations;
      }
    }

    void RestrictedModel::drawState(const State& state, cv::Mat& image) {
      base_model_->drawState(state, image);
    }

  } /* mrn */

} /* bwi_guidance */
