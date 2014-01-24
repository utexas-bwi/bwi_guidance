#include <boost/foreach.hpp>
#include <cassert>
#include <cmath>
#include <fstream>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

#include <bwi_guidance_solver/person_model_iros14.h>
#include <bwi_mapper/point_utils.h>
#include <bwi_mapper/map_utils.h>

namespace bwi_guidance {

  PersonModelIROS14::PersonModelIROS14(const bwi_mapper::Graph& graph, const
      nav_msgs::OccupancyGrid& map, size_t goal_idx, int
      action_vertex_visibility_depth, int max_robots_in_use, float
      visibility_range, bool allow_goal_visibility, float human_speed,
      float robot_speed) : graph_(graph),
  map_(map), goal_idx_(goal_idx), max_robots_in_use_(max_robots_in_use),
  allow_goal_visibility_(allow_goal_visibility), human_speed_(human_speed),
  robot_speed_(robot_speed) {

    num_vertices_ = boost::num_vertices(graph_);
    computeAdjacentVertices(adjacent_vertices_map_, graph_);
    computeVisibleVertices(visible_vertices_map_, graph_, map_, visibility_range_);

    // Compute Action Vertices
    // Start with the visible vertices and expand with adjacent vertices until needed
    action_vertices_map_ = visible_vertices_map_;
    for (int n = 0; n < action_vertex_visibility_depth; ++n) {
      for (int i = 0; i < num_vertices_; ++i) {
        std::set<int> action_vertices(action_vertices_map_[i].begin(),
            action_vertices_map_[i].end());
        BOOST_FOREACH(int vtx, action_vertices_map_[i]) {
          action_vertices.insert(adjacent_vertices_map_[vtx].begin(),
              adjacent_vertices_map_[vtx].end());
        }
        action_vertices_map_[i] = std::vector<int>(action_vertices.begin(),
            action_vertices.end());
      }
    }

  }

  bool PersonModelIROS14::isTerminalState(const StateIROS14& state) const {
    return state.graph_id == goal_idx_;
  }

  void PersonModelIROS14::getActionsAtState(const StateIROS14& state, 
      std::vector<ActionIROS14>& actions) {
    actions.clear();
    actions.push_back(ActionIROS14(DO_NOTHING, 0, 0));
    for (int i = 0; i < state.in_use_robots.size(); ++i) {
      actions.push_back(ActionIROS14(RELEASE_ROBOT, 
                                     state.in_use_robots[i].robot_id,
                                     0));
    }
    if (state.in_use_robots.size() != max_robots_in_use_) {
      BOOST_FOREACH(int vtx, action_vertices_map_[state.graph_id]) {
        BOOST_FOREACH(int adj, adjacent_vertices_map_[vtx]) {
          actions.push_back(ActionIROS14(ASSIGN_ROBOT, vtx, adj));
        }
      }
    }
  }

  float PersonModelIROS14::takeActionAtCurrentState(
      const ActionIROS14& action) {

    assert(!isTerminalState(current_state_));

    if (action.type == RELEASE_ROBOT) {
      int mark_for_removal = -1;
      for (int i = 0; i < current_state_.in_use_robots.size(); ++i) {
        if (action.at_graph_id == current_state_.in_use_robots[i].robot_id) {
          mark_for_removal = i;
          break;
        }
      }
      assert(mark_for_removal != -1);
      current_state_.in_use_robots.erase(
          current_state_.in_use_robots.begin() + mark_for_removal);
      return 0;
    }

    if (action.type == ASSIGN_ROBOT) {
      assert(current_state_.in_use_robots.size() < max_robots_in_use_);
      float distance_to_destination = bwi_mapper::getShortestPathDistance(
          current_state_.graph_id, action.at_graph_id, graph_); 
      float time_to_destination = distance_to_destination * human_speed_; 
      InUseRobotStateIROS14 r;
      r.robot_id = 
        selectBestRobotForTask(action.at_graph_id, time_to_destination);
      r.destination = action.at_graph_id;
      r.direction = action.guide_graph_id;
      current_state_.in_use_robots.push_back(r);
      return 0;
    }

    // alright, need to wait for the human to take an action - let's first
    // figure out what action he takes
    float expected_dir = getAngleInRadians(current_state_.direction);
    float robot_dir = 0;
    if (!isRobotDirectionAvailable(robot_dir)) {
      expected_dir = robot_dir;
    }
    
    // Now assume that the person moves to one the adjacent locations
    float weight_sum = 0;
    std::vector<float> weights;
    BOOST_FOREACH(int adj, adjacent_vertices_map_[current_state_.graph_id]) {

      float next_state_direction = bwi_mapper::getNodeAngle(
          current_state_.graph_id, adj, graph_);
      float angle_difference = getAbsoluteAngleDifference(next_state_direction, 
          expected_dir);

      // Compute the probability of this state
      float weight = exp(-pow(angle_difference, 2) / (2 * 0.1));
      weights.push_back(weight);
      weight_sum += weight;
    }

    float probability_sum = 0;
    std::vector<float> probabilities;
    for (size_t probability_counter = 0; probability_counter < weights.size();
        ++probability_counter) {
      float probability = 0.9 * (weights[probability_counter] / weight_sum) +
        0.1 * (1.0f / weights.size());
      probability_sum += probability;
      if (probability_counter == weights.size() - 1) {
        // Account for floating point errors. No surprises!
        probability += 1.0f - probability_sum; 
      }
      probabilities.push_back(probability);
    }

    int next_node = select(probabilities, ugen_);

    // Now that we've decided which vertex the person is moving to, compute
    // time/distance to that vertex and update all the robots
    float time_to_vertex = human_speed_ * bwi_mapper::getEuclideanDistance(
        next_node, current_state_.graph_id, graph_);

    // Transition to next state
    current_state_.direction = computeNextDirection(
        current_state_.direction, current_state_.graph_id, next_node, graph_);
    current_state_.graph_id = next_node;
    moveRobots(time_to_vertex);

    // Compute reward
    float reward = -time_to_vertex;
    // TODO how to incorporate utility? Is there any good way to add utility 
    // incrementally?
    return reward;
  }

  void PersonModelIROS14::setState(const StateIROS14 &state) {
    current_state_ = state;
  }

  void PersonModelIROS14::takeAction(const ActionIROS14 &action, float &reward, 
      StateIROS14 &state, bool &terminal) {

    assert(ugen_ != NULL && pgen_ != NULL);
    assert(!isTerminalState(current_state_));

    reward = takeActionAtCurrentState(action);
    state = current_state_;
    terminal = isTerminalState(current_state_);
  }

  void PersonModelIROS14::getFirstAction(const StateIROS14 &state, 
      ActionIROS14 &action) {
    std::vector<ActionIROS14> actions;
    getActionsAtState(state, actions);
    action = actions[0];
  }

  bool PersonModelIROS14::getNextAction(const StateIROS14 &state, 
      ActionIROS14 &action) {
    std::vector<ActionIROS14> actions;
    getActionsAtState(state, actions);
    for (size_t i = 0; i < actions.size() - 1; ++i) {
      if (actions[i] == action) {
        action = actions[i + 1];
        return true;
      }
    }
    return false;
  }
  
  void PersonModelIROS14::initializeRNG(URGenPtr ugen, PIGenPtr pgen) {
    ugen_ = ugen;
    pgen_ = pgen;
  }

} /* bwi_guidance */
