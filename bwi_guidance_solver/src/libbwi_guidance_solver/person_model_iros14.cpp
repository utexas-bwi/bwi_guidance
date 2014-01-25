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

    robot_speed_ /= map_.info.resolution;
    human_speed_ /= map_.info.resolution;

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
    std::vector<int> assigned_vertices;
    for (int i = 0; i < state.in_use_robots.size(); ++i) {
      actions.push_back(ActionIROS14(RELEASE_ROBOT, 
                                     state.in_use_robots[i].robot_id,
                                     0));
      assigned_vertices.push_back(state.in_use_robots[i].direction);
    }
    if (state.in_use_robots.size() != max_robots_in_use_) {
      BOOST_FOREACH(int vtx, action_vertices_map_[state.graph_id]) {
        if (std::find(assigned_vertices.begin(), assigned_vertices.end(), vtx) ==
            assigned_vertices.end()) {
          BOOST_FOREACH(int adj, adjacent_vertices_map_[vtx]) {
            actions.push_back(ActionIROS14(ASSIGN_ROBOT, vtx, adj));
          }
        }
      }
    }
  }

  float PersonModelIROS14::getTrueDistanceTo(const RobotStateIROS14& robot, 
      int destination) {
    std::vector<size_t> shortest_path;
    float distance = 0;
    if (robot.graph_id != destination) {
      distance = bwi_mapper::getShortestPathWithDistance(
          robot.graph_id, robot.destination, shortest_path, graph_);
    }
    shortest_path.insert(shortest_path.begin(), robot.destination);
    int next_node = (shortest_path.size() >= 2) ?
      shortest_path[shortest_path.size() - 2] : -1;
    int neighbor_node = (robot.robot_precision < 0.0f) ?
      robot.from_graph_node : next_node;
    if (neighbor_node != -1) {
      distance -= robot.robot_precision *
        bwi_mapper::getEuclideanDistance(robot.graph_id, neighbor_node, graph_);
    }
    return distance;
  }

  int PersonModelIROS14::selectBestRobotForTask(int destination, 
      float time_to_destination) {

    std::vector<float> utility_loss(current_state_.robots.size(),
        std::numeric_limits<float>::max());
    std::vector<int> robots_in_use(current_state_.in_use_robots.size());
    for (int i = 0; i < current_state_.in_use_robots.size(); ++i) {
      robots_in_use[i] = current_state_.in_use_robots[i].robot_id;
    }
    for (int i = 0; i < current_state_.robots.size(); ++i) {
      if (std::find(robots_in_use.begin(), robots_in_use.end(), i) != 
          robots_in_use.end()) {
        float orig_distance = getTrueDistanceTo(current_state_.robots[i],
            current_state_.robots[i].destination);
        float original_time = orig_distance / robot_speed_;
        float new_distance_1 = getTrueDistanceTo(current_state_.robots[i],
            destination);
        float new_distance_2 = bwi_mapper::getShortestPathDistance(
            destination,
            current_state_.robots[i].destination, graph_);
        float new_time = 
          std::max(new_distance_1 / robot_speed_, time_to_destination) + 
          new_distance_2 / robot_speed_;
        // TODO - introduce utility multiplier here
        utility_loss[i] = new_time - original_time;
      }
    }
    return std::min_element(utility_loss.begin(), utility_loss.end()) - 
      utility_loss.begin();
  }

  bool PersonModelIROS14::isRobotDirectionAvailable(float& robot_dir) {
    // Figure out if there is a robot at the current position
    for (int i = 0; i < current_state_.robots.size(); ++i) {
      bool robot_in_use = false;
      int destination, direction;
      for (int j = 0; j < current_state_.in_use_robots.size(); ++j) {
        if (current_state_.in_use_robots[j].robot_id == i) {
          destination = current_state_.in_use_robots[j].destination;
          direction = current_state_.in_use_robots[j].direction;
          robot_in_use = true;
          break;
        }
      }
      if (!robot_in_use || destination != current_state_.graph_id)
        continue;
      int current_robot_id = current_state_.robots[i].graph_id;
      if (current_robot_id == current_state_.graph_id &&
          current_state_.robots[i].robot_precision >= 0.0f) {
        robot_dir = 
          bwi_mapper::getNodeAngle(current_robot_id, direction, graph_);
        return true;
      }
    }
    
    return false;
  }

  int PersonModelIROS14::generateNewGoalFrom(int idx) {
    // TODO
    return rand() % num_vertices_;
    // assert(pgen_);
    // int graph_distance = (*pgen_)();
    // for (int i = 0; i < num_vertices_; ++i) {



    // }
  }

  void PersonModelIROS14::moveRobots(float time) {
    for (int i = 0; i < current_state_.robots.size(); ++i) {
      int& current_graph_id = current_state_.robots[i].graph_id;
      int& destination = current_state_.robots[i].destination; 
      bool robot_in_use = false;
      for (int j = 0; j < current_state_.in_use_robots.size(); ++j) {
        if (current_state_.in_use_robots[j].robot_id == i) {
          destination = current_state_.in_use_robots[j].destination;
          robot_in_use = true;
          break;
        }
      }

      std::vector<size_t> shortest_path;
      if (current_graph_id != destination) {
        getShortestPathWithDistance(
            current_graph_id, destination, shortest_path, graph_);
      }
      shortest_path.insert(shortest_path.begin(), destination);
      float coverable_distance = time * robot_speed_;
      float& current_precision = current_state_.robots[i].robot_precision; 
      int& old_graph_id = current_state_.robots[i].from_graph_node;
      int next_node_counter = (int)shortest_path.size() - 2;
      int next_node_id = -1;
      if (next_node_counter >= 0) {
        next_node_id = shortest_path[next_node_counter];
      }
      while (coverable_distance > 0.0f) {
        if (current_precision < 0.0f) {
          float edge_distance = bwi_mapper::getEuclideanDistance(
              current_graph_id, old_graph_id, graph_);
          current_precision += coverable_distance / edge_distance;
          coverable_distance = current_precision * edge_distance; 
          current_precision = 
            std::max(0.0f, current_precision);
        } else if (next_node_id != -1) {
          float edge_distance = bwi_mapper::getEuclideanDistance(
              current_graph_id, next_node_id, graph_);
          current_precision += coverable_distance / edge_distance;
          coverable_distance = (current_precision - 0.5f) * edge_distance; 
          if (coverable_distance > 0.0f) {
            current_precision = -0.5f;
            old_graph_id = current_graph_id;
            current_graph_id = next_node_id;
            --next_node_counter;
            next_node_id = (next_node_counter >= 0) ?
              shortest_path[next_node_counter] : -1;
          }
        }
      }

      // If a robot is not in use, generate a new goal move towards it
      if (!robot_in_use) {
        destination = generateNewGoalFrom(destination);
        // Reprocess this robot
        --i;
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
      float time_to_destination = distance_to_destination / human_speed_; 
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
    float time_to_vertex = bwi_mapper::getEuclideanDistance(
        next_node, current_state_.graph_id, graph_) / human_speed_;

    // Transition to next state
    current_state_.direction = computeNextDirection(
        current_state_.direction, current_state_.graph_id, next_node, graph_);
    current_state_.graph_id = next_node;

    // TODO allow moving robots and people slowly for visualization
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

    assert(ugen_);
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
