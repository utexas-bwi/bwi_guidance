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
      nav_msgs::OccupancyGrid& map, size_t goal_idx, int max_robots, int
      max_robots_in_use, int action_vertex_visibility_depth, float
      visibility_range, bool allow_goal_visibility, float human_speed, float
      robot_speed) : graph_(graph),
  map_(map), goal_idx_(goal_idx), max_robots_(max_robots), max_robots_in_use_(max_robots_in_use),
  allow_goal_visibility_(allow_goal_visibility), human_speed_(human_speed),
  robot_speed_(robot_speed), initialized_(false) {

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

    cacheNewGoalsByDistance();
    cacheShortestPaths();

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
    // Optimized!!!
    float distance = shortest_distances_[robot.graph_id][destination];
    std::vector<size_t>& shortest_path =
      shortest_paths_[robot.graph_id][destination];
    int next_node = (shortest_path.size() > 0) ?  shortest_path[0] : -1;
    int neighbor_node = (robot.precision < 0.0f) ?
      robot.from_graph_node : next_node;
    if (neighbor_node != -1) {
      distance -= robot.precision *
        shortest_distances_[robot.graph_id][neighbor_node];
    }
    return distance;
  }

  int PersonModelIROS14::selectBestRobotForTask(int destination, 
      float time_to_destination) {

    std::vector<float> utility_loss(current_state_.robots.size(),
        std::numeric_limits<float>::max());
    for (int i = 0; i < current_state_.robots.size(); ++i) {
      bool robot_in_use = false;
      for (int j = 0; j < current_state_.in_use_robots.size(); ++j) {
        if (current_state_.in_use_robots[j].robot_id == i) {
          destination = current_state_.in_use_robots[j].destination;
          robot_in_use = true;
          break;
        }
      }
      if (!robot_in_use) {
        float orig_distance = getTrueDistanceTo(current_state_.robots[i],
            current_state_.robots[i].destination);
        float original_time = orig_distance / robot_speed_;
        float new_distance_1 = getTrueDistanceTo(current_state_.robots[i],
            destination);
        float new_distance_2 = 
          shortest_distances_[destination][current_state_.robots[i].destination];
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
          current_state_.robots[i].precision >= 0.0f) {
        robot_dir = 
          bwi_mapper::getNodeAngle(current_robot_id, direction, graph_);
        return true;
      }
    }
    
    return false;
  }

  void PersonModelIROS14::cacheShortestPaths() {
    shortest_paths_.clear();
    shortest_distances_.clear();
    shortest_paths_.resize(num_vertices_);
    shortest_distances_.resize(num_vertices_);
    for (int idx = 0; idx < num_vertices_; ++idx) {
      /* std::cout << "For vtx " << idx << ":" << std::endl; */
      shortest_distances_[idx].resize(num_vertices_);
      shortest_paths_[idx].resize(num_vertices_);
      for (int j = 0; j < num_vertices_; ++j) {
        if (j == idx) {
          shortest_distances_[idx][j] = 0;
          shortest_paths_[idx][j].clear();
        } else {
          shortest_distances_[idx][j] = bwi_mapper::getShortestPathWithDistance(
              idx, j, shortest_paths_[idx][j], graph_);

          // Post-process the shortest path - add goal, remove start and reverse
          shortest_paths_[idx][j].insert(shortest_paths_[idx][j].begin(), j);
          shortest_paths_[idx][j].pop_back();
          std::reverse(shortest_paths_[idx][j].begin(), 
              shortest_paths_[idx][j].end());
        }

        // std::cout << "  To " << j << "(" << shortest_distances_[idx][j] << 
        //   "): ";
        // BOOST_FOREACH(int c, shortest_paths_[idx][j]) {
        //   std::cout << c << " "; 
        // }
        // std::cout << std::endl;
      }
    }
  }

  void PersonModelIROS14::cacheNewGoalsByDistance() {
    goals_by_distance_.clear();
    goals_by_distance_.resize(num_vertices_);
    for (int idx = 0; idx < num_vertices_; ++idx) {
      // Use BFS to save all graph nodes realtive to this one by number of edges
      // std::cout << "For vtx " << idx << ":" << std::endl;
      std::set<int> closed_set;
      std::vector<int> current_set;
      current_set.push_back(idx);
      // int distance = 0;
      while (current_set.size() != 0) {
        // std::cout << "  At distance " << distance << ": ";
        // BOOST_FOREACH(int c, current_set) {
        //   std::cout << c << " "; 
        // }
        // std::cout << std::endl;
        // ++distance;
        goals_by_distance_[idx].push_back(current_set);
        std::set<int> open_set;
        BOOST_FOREACH(int c, current_set) {
          BOOST_FOREACH(int a, adjacent_vertices_map_[c]) {
            if (std::find(closed_set.begin(), closed_set.end(), a) ==
                closed_set.end()) {
              open_set.insert(a);
            }
          }
        }
        closed_set.insert(current_set.begin(), current_set.end());
        current_set = std::vector<int>(open_set.begin(), open_set.end());
      }
    }
  }

  int PersonModelIROS14::generateNewGoalFrom(int idx) {
    // Optimized!!!
    assert(pgen_ && uigen_ && goals_by_distance_.size() == num_vertices_);
    while(true) {
      int graph_distance = (*pgen_)();
      if (graph_distance == 0 || 
          graph_distance >= goals_by_distance_[idx].size()) {
        continue;
      }
      std::vector<int>& possible_goals = 
        goals_by_distance_[idx][graph_distance];
      return *(possible_goals.begin() + ((*uigen_)() % possible_goals.size()));
    }
  }

  void PersonModelIROS14::moveRobots(float time) {
    std::cout << "Moving ahead for " << time << " seconds" << std::endl;
    // Optimized!!!
    for (int i = 0; i < current_state_.robots.size(); ++i) {
      RobotStateIROS14& robot = current_state_.robots[i];

      // Check if we are moving this robot under our control or not
      int destination = robot.destination;
      bool robot_in_use = false;
      for (int j = 0; j < current_state_.in_use_robots.size(); ++j) {
        if (current_state_.in_use_robots[j].robot_id == i) {
          destination = current_state_.in_use_robots[j].destination;
          robot_in_use = true;
          break;
        }
      }

      // Get shortest path to destination, and figure out how much distance
      // of that path we can cover
      std::vector<size_t>* shortest_path =
        &(shortest_paths_[robot.graph_id][destination]);
      float coverable_distance = time * robot_speed_;
      int next_node_counter = 0;
      int next_node_id = (next_node_counter < shortest_path->size()) ?
        (*shortest_path)[next_node_counter] : -1;
      while (coverable_distance > 0.0f) {
        if (robot.precision < 0.0f) {
          // Moving to robot.graph_id
          float edge_distance =
            shortest_distances_[robot.graph_id][robot.from_graph_node];
          assert(edge_distance != 0);
          robot.precision += coverable_distance / edge_distance;
          coverable_distance = robot.precision * edge_distance; 
          robot.precision = std::min(0.0f, robot.precision);
        } else if (next_node_id != -1) {
          // Moving away from robot.graph_id
          float edge_distance = 
            shortest_distances_[robot.graph_id][next_node_id];
          assert(edge_distance != 0);
          robot.precision += coverable_distance / edge_distance;
          coverable_distance = (robot.precision - 0.5f) * edge_distance; 
          if (coverable_distance > 0.0f) {
            // Move to next section of
            robot.precision = -0.5f;
            robot.from_graph_node = robot.graph_id;
            robot.graph_id = next_node_id;
            ++next_node_counter;
            next_node_id = (next_node_counter < shortest_path->size()) ?
              (*shortest_path)[next_node_counter] : -1;
          }
        } else {
          // The robot has reached the destination
          if (robot_in_use) {
            // Won't be doing anything more until the robot gets released
            coverable_distance = 0.0f;
          } else {
            // Assign new goal and move towards that goal
            robot.destination = generateNewGoalFrom(destination);
            shortest_path =
              &(shortest_paths_[robot.graph_id][robot.destination]);
            next_node_counter = 0;
            next_node_id = (next_node_counter < shortest_path->size()) ?
              (*shortest_path)[next_node_counter] : -1;
          }
        }
      }

    }
  }

  float PersonModelIROS14::takeActionAtCurrentState(
      const ActionIROS14& action) {

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
    if (isRobotDirectionAvailable(robot_dir)) {
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

    int next_node = adjacent_vertices_map_[current_state_.graph_id]
      [select(probabilities, ugen_)];

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
    if (current_state_.robots.size() == 0) {
      assert(current_state_.in_use_robots.size() == 0);
      addRobots(max_robots_);
    }
    initialized_ = true;
  }

  void PersonModelIROS14::takeAction(const ActionIROS14 &action, float &reward, 
      StateIROS14 &state, bool &terminal) {

    assert(initialized_);
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

  void PersonModelIROS14::addRobots(int n) {
    assert(uigen_);
    for (int r = 0; r < n; ++r) {
      RobotStateIROS14 robot;
      robot.graph_id = (*uigen_)();
      robot.destination = generateNewGoalFrom(robot.graph_id); 
      robot.precision = 0.0f;
      current_state_.robots.push_back(robot);
    }
  }

  void PersonModelIROS14::initializeRNG(UIGenPtr uigen, URGenPtr ugen, 
      PIGenPtr pgen) {
    uigen_ = uigen;
    ugen_ = ugen;
    pgen_ = pgen;
  }

  void PersonModelIROS14::drawCurrentState(cv::Mat& image) {
    // TODO OPTIMIZE!!!
    assert(initialized_);
    bwi_mapper::drawCircleOnGraph(image, graph_, current_state_.graph_id);
    for (int r = 0; r < current_state_.robots.size(); ++r) {
      RobotStateIROS14& robot = current_state_.robots[r];
      cv::Scalar color((r * 12345) % 128, (r * 23456) % 128, (r * 34567) % 128);
      cv::Point2f robot_pos;
      if (robot.precision < 0.0f) {
        robot_pos = 
          -robot.precision * bwi_mapper::getLocationFromGraphId(robot.from_graph_node, graph_) + 
          (1 + robot.precision) * bwi_mapper::getLocationFromGraphId(robot.graph_id, graph_);
      } else {
        int next_node = robot.graph_id;
        std::vector<size_t> shortest_path;
        if (robot.graph_id != robot.destination) {
          bwi_mapper::getShortestPathWithDistance(robot.graph_id, robot.destination, shortest_path, graph_);
        }
        shortest_path.insert(shortest_path.begin(), robot.destination);
        if (shortest_path.size() >= 2) {
          next_node = shortest_path[shortest_path.size() - 2];
        }
        robot_pos = 
          (1 - robot.precision) * bwi_mapper::getLocationFromGraphId(robot.graph_id, graph_) + 
          (robot.precision) * bwi_mapper::getLocationFromGraphId(next_node, graph_);
      }
      cv::circle(image, robot_pos, 10, color, -1);
      bwi_mapper::drawSquareOnGraph(image, graph_, robot.destination, color);
    }
  }

  void PersonModelIROS14::printDistanceToDestination(int idx) {
    assert(idx >= 0 && idx < current_state_.robots.size());
    std::cout << "Distance of Robot " << idx << " to its destination: " <<
      getTrueDistanceTo(current_state_.robots[idx],
          current_state_.robots[idx].destination) << std::endl;
  }
} /* bwi_guidance */
