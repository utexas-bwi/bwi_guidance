#include <boost/foreach.hpp>
#include <cassert>
#include <cmath>
#include <fstream>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

#include <bwi_guidance_solver/person_model_iros14.h>
#include <bwi_mapper/point_utils.h>
#include <bwi_mapper/map_utils.h>

namespace bwi_guidance {

  PersonModelIROS14::PersonModelIROS14(const bwi_mapper::Graph& graph, 
      const nav_msgs::OccupancyGrid& map, size_t goal_idx, float frame_rate,
      int max_robots_in_use, int action_vertex_visibility_depth, 
      int action_vertex_adjacency_depth, float visibility_range, 
      bool allow_goal_visibility, float human_speed, float robot_speed) :
    graph_(graph), map_(map), goal_idx_(goal_idx),
    frame_rate_(frame_rate), max_robots_in_use_(max_robots_in_use),
    allow_goal_visibility_(allow_goal_visibility), human_speed_(human_speed),
    robot_speed_(robot_speed), initialized_(false) {

    robot_speed_ /= map_.info.resolution;
    human_speed_ /= map_.info.resolution;
    visibility_range /= map_.info.resolution;

    num_vertices_ = boost::num_vertices(graph_);
    computeAdjacentVertices(adjacent_vertices_map_, graph_);
    computeVisibleVertices(visible_vertices_map_, graph_, map_, visibility_range);

    // Compute Action Vertices

    // Start with the visible vertices and expand to depth
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

    // Also add adjacent vertices based on depth
    for (int idx = 0; idx < num_vertices_; ++idx) {
      std::set<int> action_vertices(action_vertices_map_[idx].begin(),
          action_vertices_map_[idx].end());
      std::set<int> closed_set;
      std::set<int> current_set;
      current_set.insert(idx);
      for (int n = 0; 
          current_set.size() != 0 && n <= action_vertex_adjacency_depth; ++n) {
        action_vertices.insert(current_set.begin(), current_set.end());
        std::set<int> open_set;
        BOOST_FOREACH(int c, current_set) {
          BOOST_FOREACH(int a, adjacent_vertices_map_[c]) {
            if (closed_set.find(a) == closed_set.end()) {
              open_set.insert(a);
            }
          }
        }
        closed_set.insert(current_set.begin(), current_set.end());
        current_set = open_set;
      }
      action_vertices_map_[idx] = std::vector<int>(action_vertices.begin(),
          action_vertices.end());
    }

    // std::cout << "Action Vertices Map:" << std::endl;
    // for (int i = 0; i < num_vertices_; ++i) {
    //   std::cout << " For vtx " << i << ": ";
    //   BOOST_FOREACH(int vtx, action_vertices_map_[i]) {
    //     std::cout << vtx << " ";
    //   }
    //   std::cout << std::endl;
    // }

    /* exit(0); */

    cacheNewGoalsByDistance();
    cacheShortestPaths();

  }

  bool PersonModelIROS14::isTerminalState(const StateIROS14& state) const {
    return state.graph_id == goal_idx_;
  }

  void PersonModelIROS14::getActionsAtState(const StateIROS14& state, 
      std::vector<ActionIROS14>& actions) {
    actions.clear();
    int dir;
    if (isRobotDirectionAvailable(state, dir)) {
      if (dir == DIR_UNASSIGNED) {
        BOOST_FOREACH(int adj, adjacent_vertices_map_[state.graph_id]) {
          actions.push_back(ActionIROS14(GUIDE_PERSON, state.graph_id, adj));
        }
        return; // Only choose a direction here
      }
    }
    actions.push_back(ActionIROS14(DO_NOTHING, 0, 0));
    std::vector<int> cant_assign_vertices = state.relieved_locations;
    for (int i = 0; i < state.in_use_robots.size(); ++i) {
      if (std::find(state.acquired_locations.begin(),
            state.acquired_locations.end(),
            state.in_use_robots[i].destination) ==
          state.acquired_locations.end()) {
        actions.push_back(ActionIROS14(RELEASE_ROBOT, 
                                       state.in_use_robots[i].destination,
                                       0));
      }
      cant_assign_vertices.push_back(state.in_use_robots[i].destination);
    }
    if (state.in_use_robots.size() != max_robots_in_use_) {
      BOOST_FOREACH(int vtx, action_vertices_map_[state.graph_id]) {
        if (std::find(cant_assign_vertices.begin(), 
                      cant_assign_vertices.end(), vtx) ==
            cant_assign_vertices.end()) {
          /* BOOST_FOREACH(int adj, adjacent_vertices_map_[vtx]) { */
          actions.push_back(ActionIROS14(ASSIGN_ROBOT, vtx, DIR_UNASSIGNED));
          /* } */
        }
      }
    }
  }

  float PersonModelIROS14::getTrueDistanceTo(RobotStateIROS14& robot, 
      int current_destination, int to_destination, bool change_robot_state) {

    /* std::cout << "in here1: " << robot.graph_id << " " << robot.precision << " " << change_robot_state << std::endl; */
    // Optimized!!!
    float ret_distance;
    float current_edge_distance = 
      shortest_distances_[robot.graph_id][robot.other_graph_node];
    if (robot.precision < 0.5f) {
      float distance_through_current_node = 
        robot.precision * current_edge_distance
        + shortest_distances_[robot.graph_id][to_destination];
      float distance_through_other_node = 
        (1.0f - robot.precision) * current_edge_distance
        + shortest_distances_[robot.other_graph_node][to_destination];
      if (distance_through_current_node <= distance_through_other_node) {
        ret_distance = distance_through_current_node;
        // The robot should be flipped around
        if (change_robot_state) {
          /* std::cout << "in here" << robot.precision << " " << robot.graph_id << " " << robot.other_graph_node << " " << to_destination << std::endl; */
          // The destination should have changed externally already
          if (robot.precision != 0.0f) {
            robot.precision = 1.0f - robot.precision;
          } else {
            // The robot is exactly at robot.graph_id, find shortest path to
            // destination
            std::vector<size_t>& shortest_path = 
              shortest_paths_[robot.graph_id][to_destination];
            if (shortest_path.size() > 0) {
              robot.other_graph_node = shortest_path[0];
            } else {
              robot.other_graph_node = robot.graph_id;
            }
          }
          /* std::cout << "in here" << robot.precision << " " << robot.graph_id << " " << robot.other_graph_node << " " << to_destination << std::endl; */

        }
      } else {
        ret_distance = distance_through_other_node;
        // Nothing needs to change for optimal solution
      }
    } else {
      float distance_through_current_node = 
        (1.0f - robot.precision) * current_edge_distance
        + shortest_distances_[robot.graph_id][to_destination];
      float distance_through_other_node = 
        robot.precision * current_edge_distance
        + shortest_distances_[robot.other_graph_node][to_destination];
      if (distance_through_current_node <= distance_through_other_node) {
        ret_distance = distance_through_current_node;
        // Nothing needs to change for optimal solution
      } else {
        ret_distance = distance_through_other_node;
        // The robot should be flipped around
        if (change_robot_state) {
          /* std::cout << "in here2" << std::endl; */
          // The destination should have changed externally already
          robot.precision = 1.0f - robot.precision;
        }
      }
    }
    return ret_distance;
  }
  void PersonModelIROS14::changeRobotDirectionIfNeeded(RobotStateIROS14& state, 
      int current_destination, int to_destination) {
    // This will flip the robot around based on the optimal path
    getTrueDistanceTo(state, current_destination, to_destination, true);
  }

  int PersonModelIROS14::selectBestRobotForTask(int destination, 
      float time_to_destination) {

    std::vector<float> utility_loss_all(current_state_.robots.size(),
        std::numeric_limits<float>::max());
    std::vector<float> utility_loss_in_time(current_state_.robots.size(),
        std::numeric_limits<float>::max());
    std::vector<float> time(current_state_.robots.size(),
        std::numeric_limits<float>::max());

    for (int i = 0; i < current_state_.robots.size(); ++i) {
      bool robot_in_use = false;
      for (int j = 0; j < current_state_.in_use_robots.size(); ++j) {
        if (current_state_.in_use_robots[j].robot_id == i) {
          robot_in_use = true;
          break;
        }
      }
      if (!robot_in_use) {
        float orig_distance = getTrueDistanceTo(current_state_.robots[i],
            current_state_.robots[i].destination, 
            current_state_.robots[i].destination);
        float original_time = orig_distance / robot_speed_;
        float new_distance_1 = getTrueDistanceTo(current_state_.robots[i],
            current_state_.robots[i].destination,
            destination);
        float new_distance_2 = 
          shortest_distances_[destination][current_state_.robots[i].destination];
        float new_time = 
          std::max(new_distance_1 / robot_speed_, time_to_destination) + 
          new_distance_2 / robot_speed_;
        // TODO - introduce utility multiplier here
        utility_loss_all[i] = new_time - original_time;
        if (new_distance_1 / robot_speed_ <= time_to_destination) {
          // The robot will reach there in time
          utility_loss_in_time[i] = new_time - original_time;
        }
        time[i] = new_distance_1 / robot_speed_;
      }
    }
    
    if (*(std::min_element(utility_loss_in_time.begin(), utility_loss_in_time.end())) != 
          std::numeric_limits<float>::max()) {
      return std::min_element(utility_loss_in_time.begin(), utility_loss_in_time.end()) - 
        utility_loss_in_time.begin();
    } else {
      return std::min_element(time.begin(), time.end()) - time.begin();
    }

  }

  bool PersonModelIROS14::isRobotDirectionAvailable(const StateIROS14& state,
      int& robot_dir) {
    // Figure out if there is a robot at the current position
    for (int i = 0; i < state.in_use_robots.size(); ++i) {
      int destination = state.in_use_robots[i].destination;
      int direction = state.in_use_robots[i].direction;
      int robot_graph_id = state.robots[state.in_use_robots[i].robot_id].graph_id;
      bool reached_destination = state.in_use_robots[i].reached_destination;
      if (destination != state.graph_id ||
          robot_graph_id != state.graph_id ||
          !reached_destination)
        continue;
      robot_dir = direction;
        //bwi_mapper::getNodeAngle(current_robot_id, direction, graph_);
      return true;
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
      if (graph_distance >= goals_by_distance_[idx].size()) {
        continue;
      }
      std::vector<int>& possible_goals = 
        goals_by_distance_[idx][graph_distance];
      return *(possible_goals.begin() + ((*uigen_)() % possible_goals.size()));
    }
  }

  bool PersonModelIROS14::moveRobots(float time) {
    // Move the human
    bool ready_for_next_action = true;
    if (frame_rate_ > 0.0f) {
      float human_coverable_distance = time * human_speed_;
      float human_edge_distance = shortest_distances_[current_state_.from_graph_node][current_state_.graph_id];
      float added_precision = human_coverable_distance / human_edge_distance;
      current_state_.precision += added_precision;
      if (current_state_.precision < 1.0f) {
        ready_for_next_action = false;
      } else {
        time -= (current_state_.precision - 1.0f) * human_edge_distance / human_speed_;
        current_state_.precision = 1.0f;
      }
    }
    
    /* std::cout << "Moving ahead for " << time << " seconds" << std::endl; */
    // Optimized!!!
    for (int i = 0; i < current_state_.robots.size(); ++i) {
      RobotStateIROS14& robot = current_state_.robots[i];

      // Check if we are moving this robot under our control or not
      int destination = robot.destination;
      bool robot_in_use = false;
      bool* robot_reached = NULL;
      for (int j = 0; j < current_state_.in_use_robots.size(); ++j) {
        if (current_state_.in_use_robots[j].robot_id == i) {
          destination = current_state_.in_use_robots[j].destination;
          robot_reached = &(current_state_.in_use_robots[j].reached_destination);
          robot_in_use = true;
          break;
        }
      }

      // Get shortest path to destination, and figure out how much distance
      // of that path we can cover
      float coverable_distance = time * robot_speed_;
      while (coverable_distance > 0.0f) {
        /* std::cout << robot.graph_id << " " << robot.precision << " " << destination << " " << robot.other_graph_node << std::endl; */
        if (robot.precision == 0.0f && robot.graph_id == destination) {
          // The robot has reached its destination
          if (robot_in_use) {
            // Won't be doing anything more until the robot gets released
            coverable_distance = 0.0f;
            *robot_reached = true;
          } else {
            // Assign new goal and move towards that goal
            robot.destination = generateNewGoalFrom(ROBOT_HOME_BASE[i]);
            destination = robot.destination;
            std::vector<size_t>& shortest_path =
              shortest_paths_[robot.graph_id][robot.destination];
            if (shortest_path.size() > 0) {
              // This means that robot.graph_id != new goal
              robot.other_graph_node = shortest_path[0];
            } else {
              robot.other_graph_node = robot.graph_id;
            }
          }
        } else {
          float current_edge_distance = 
            shortest_distances_[robot.graph_id][robot.other_graph_node];
          if (current_edge_distance == 0.0f) {
            std::cout << robot.graph_id << " " << robot.precision << " " << destination << " " << robot.other_graph_node << std::endl;
            exit(-1);
          }
          float added_precision = coverable_distance / current_edge_distance;
          if (robot.precision < 0.5f) {
            // Moving away from robot.graph_id
            if (robot.precision + added_precision >= 0.5f) {
              coverable_distance -= (0.5f - robot.precision) * current_edge_distance;
              robot.precision = 0.5f;
              std::swap(robot.graph_id, robot.other_graph_node);
            } else {
              robot.precision += added_precision;
              coverable_distance = 0.0f;
            }
          } else {
            // Moving to robot.graph_id
            if (robot.precision + added_precision >= 1.0f) {
              /* std::cout << "in here" << std::endl; */
              // Move to next section of shortest path to goal
              coverable_distance -= (1.0f - robot.precision) * current_edge_distance;
              robot.precision = 0.0f;
              std::vector<size_t>& shortest_path =
                shortest_paths_[robot.graph_id][destination];
              if (shortest_path.size() > 0) {
                // This means that robot.graph_id != new goal
                robot.other_graph_node = shortest_path[0];
              } else {
                robot.other_graph_node = robot.graph_id;
              }
            } else {
              robot.precision += added_precision;
              coverable_distance = 0.0f;
            }
          }
        }
      }

    }

    return ready_for_next_action;
  }

  float PersonModelIROS14::takeActionAtCurrentState(
      const ActionIROS14& action) {
    /* std::cout << current_state_ << action << std::endl; */

    if (action.type == RELEASE_ROBOT) {
      int mark_for_removal = -1;
      int robot_id;
      for (int i = 0; i < current_state_.in_use_robots.size(); ++i) {
        if (action.at_graph_id == current_state_.in_use_robots[i].destination) {
          robot_id = current_state_.in_use_robots[i].robot_id;
          mark_for_removal = i;
          break;
        }
      }
      // if (mark_for_removal == -1) {
      //   std::cout << "State: " << current_state_ << std::endl;
      //   std::cout << "Action: " << action << std::endl;
      // }
      assert(mark_for_removal != -1);
      current_state_.in_use_robots.erase(
          current_state_.in_use_robots.begin() + mark_for_removal);
      current_state_.relieved_locations.push_back(action.at_graph_id);

      // Since we are changing destinations, we need to reset robot state
      // to take optimal path to goal
      RobotStateIROS14& robot = current_state_.robots[robot_id];
      changeRobotDirectionIfNeeded(robot, action.at_graph_id, robot.destination);
      return 0.0;
    }

    if (action.type == ASSIGN_ROBOT) {
      /* std::cout << current_state_ << std::endl; */
      assert(current_state_.in_use_robots.size() < max_robots_in_use_);
      float distance_to_destination = shortest_distances_[current_state_.graph_id][action.at_graph_id];
      float time_to_destination = distance_to_destination / human_speed_; 
      InUseRobotStateIROS14 r;
      r.robot_id = 
        selectBestRobotForTask(action.at_graph_id, time_to_destination);
      r.destination = action.at_graph_id;
      r.direction = DIR_UNASSIGNED;
      r.reached_destination = 
        current_state_.robots[r.robot_id].graph_id == r.destination &&
        current_state_.robots[r.robot_id].precision == 0.0f;
      current_state_.in_use_robots.push_back(r);
      current_state_.acquired_locations.push_back(action.at_graph_id);

      RobotStateIROS14& robot = current_state_.robots[r.robot_id];
      changeRobotDirectionIfNeeded(robot, robot.destination, action.at_graph_id);
      
      /* std::cout << current_state_ << std::endl; */
      // TODO return utility loss
      return -10.0;//0.0;//-20.0;
    }

    if (action.type == GUIDE_PERSON) {
      int mark = -1;
      int robot_id;
      for (int i = 0; i < current_state_.in_use_robots.size(); ++i) {
        if (action.at_graph_id == current_state_.in_use_robots[i].destination) {
          robot_id = current_state_.in_use_robots[i].robot_id;
          mark = i;
          break;
        }
      }
      assert(mark != -1);
      //current_state_.in_use_robots[mark].direction = action.guide_graph_id;
      float direction = bwi_mapper::getNodeAngle(
          current_state_.graph_id, action.guide_graph_id, graph_);
      current_state_.direction = getDiscretizedAngle(direction);
      current_state_.in_use_robots.erase(
          current_state_.in_use_robots.begin() + mark);
      current_state_.relieved_locations.push_back(action.at_graph_id);

      // Since we are changing destinations, we need to reset robot state
      // to take optimal path to goal
      RobotStateIROS14& robot = current_state_.robots[robot_id];
      changeRobotDirectionIfNeeded(robot, action.at_graph_id, robot.destination);
      return 0.0;
    }

    // alright, need to wait for the human to take an action - let's first
    // figure out what action he takes
    float expected_dir = getAngleInRadians(current_state_.direction);
    int robot_dir = 0;
    if (isRobotDirectionAvailable(current_state_, robot_dir)) {
      //assert(robot_dir != DIR_UNASSIGNED);
      if (robot_dir != DIR_UNASSIGNED) {
        expected_dir = bwi_mapper::getNodeAngle(
          current_state_.graph_id, robot_dir, graph_);
      } else {
        std::cout << "oh no!" << std::endl;
        exit(-1);
      }
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
    /* std::cout << "Transition probabilities: " << std::endl; */
    for (size_t probability_counter = 0; probability_counter < weights.size();
        ++probability_counter) {
      /* std::cout << weights[probability_counter] << " " << weight_sum << std::endl; */
      float probability = 1.0 * (weights[probability_counter] / weight_sum) +
        0.0 * (1.0f / weights.size());
      probability_sum += probability;
      if (probability_counter == weights.size() - 1) {
        // Account for floating point errors. No surprises!
        probability += 1.0f - probability_sum; 
      }
      probabilities.push_back(probability);
      // std::cout << "  to " << 
      //   adjacent_vertices_map_[current_state_.graph_id][probability_counter] <<
      //   ": " << probability << std::endl;
    }

    int next_node = adjacent_vertices_map_[current_state_.graph_id]
      [select(probabilities, ugen_)];

    // Now that we've decided which vertex the person is moving to, compute
    // time/distance to that vertex and update all the robots
    float time_to_vertex = 
      shortest_distances_[current_state_.graph_id][next_node] / human_speed_;

    // Transition to next state
    float distance_closed = 
      shortest_distances_[current_state_.graph_id][goal_idx_] -
      shortest_distances_[next_node][goal_idx_];

    current_state_.direction = computeNextDirection(
        current_state_.direction, current_state_.graph_id, next_node, graph_);
    current_state_.precision = 0.0f;
    current_state_.from_graph_node = current_state_.graph_id;
    current_state_.graph_id = next_node;
    current_state_.acquired_locations.clear();
    current_state_.relieved_locations.clear();

    if (frame_rate_ == 0.0f) {
      moveRobots(time_to_vertex);
    } else {
      assert(frame_vector_);
      frame_vector_->clear();
      while (!moveRobots(1.0f/frame_rate_)) {
        frame_vector_->push_back(current_state_);
      }
    }

    // Compute reward
    float reward = -time_to_vertex;
    reward += distance_closed / human_speed_;

    // TODO how to incorporate utility? 
    //   - At the time of release would be easy and correct, but then the 
    //     robots won't get released
    //   - At the time of acquire would only be estimated
    //   - Any way to do this incrementally, but still get correct value (best)
    return reward;
  }

  void PersonModelIROS14::setState(const StateIROS14 &state) {
    assert(state.robots.size() != 0);
    current_state_ = state;
    initialized_ = true;
  }

  void PersonModelIROS14::takeAction(const ActionIROS14 &action, float &reward, 
      StateIROS14 &state, bool &terminal, int &depth_count) {

    //boost::posix_time::ptime mst1 = boost::posix_time::microsec_clock::local_time();

    assert(initialized_);
    assert(ugen_);
    assert(!isTerminalState(current_state_));

    reward = takeActionAtCurrentState(action);
    state = current_state_;
    terminal = isTerminalState(current_state_);

    depth_count = (action.type != DO_NOTHING) ? 0 : (int)(-reward); 

    // boost::posix_time::ptime mst2 = boost::posix_time::microsec_clock::local_time();
    // std::cout << "Time elapsed: " << (mst2 - mst1).total_microseconds() << 
    //   std::endl;
  }

  void PersonModelIROS14::getFirstAction(const StateIROS14 &state, 
      ActionIROS14 &action) {
    // Optimized!
    get_action_state_ = state;
    getActionsAtState(state, get_actions_);
    action = get_actions_[0];
    get_actions_counter_ = 1;
  }

  bool PersonModelIROS14::getNextAction(const StateIROS14 &state, 
      ActionIROS14 &action) {
    assert(state == get_action_state_);
    if (get_actions_counter_ < get_actions_.size()) {
      action = get_actions_[get_actions_counter_++];
      return true;
    }
    return false;
    // bool return_next = action.type == DO_NOTHING;
    // std::vector<int> assigned_vertices(state.in_use_robots.size());
    // for (int i = 0; i < state.in_use_robots.size(); ++i) {
    //   ActionIROS14 next_action(RELEASE_ROBOT, 
    //                            state.in_use_robots[i].destination, 0);
    //   assigned_vertices[i] = state.in_use_robots[i].destination;
    //   if (return_next) {
    //     action = next_action;
    //     return true;
    //   }
    //   return_next = action == next_action;
    // }
    // if (state.in_use_robots.size() != max_robots_in_use_) {
    //   BOOST_FOREACH(int vtx, action_vertices_map_[state.graph_id]) {
    //     if (std::find(assigned_vertices.begin(), assigned_vertices.end(), vtx) ==
    //         assigned_vertices.end()) {
    //       BOOST_FOREACH(int adj, adjacent_vertices_map_[vtx]) {
    //         ActionIROS14 next_action(ASSIGN_ROBOT, vtx, adj);
    //         if (return_next) {
    //           action = next_action;
    //           return true;
    //         }
    //         return_next = action == next_action;
    //       }
    //     }
    //   }
    // }
    // return false;
  }

  void PersonModelIROS14::addRobots(StateIROS14& state, int n) {
    assert(uigen_);
    for (int r = 0; r < n; ++r) {
      RobotStateIROS14 robot;
      robot.graph_id = ROBOT_HOME_BASE[r];
      robot.destination = generateNewGoalFrom(robot.graph_id); 
      robot.precision = 0.0f;
      if (shortest_paths_[robot.graph_id][robot.destination].size() != 0) {
        robot.other_graph_node = shortest_paths_[robot.graph_id][robot.destination][0];
      } else {
        robot.other_graph_node = robot.graph_id;
      }
      state.robots.push_back(robot);
    }
  }

  void PersonModelIROS14::initializeRNG(UIGenPtr uigen, URGenPtr ugen, 
      PIGenPtr pgen) {
    uigen_ = uigen;
    ugen_ = ugen;
    pgen_ = pgen;
  }

  void PersonModelIROS14::drawState(const StateIROS14& state, cv::Mat& image) {
    assert(initialized_);

    std::vector<std::pair<cv::Point2f, cv::Scalar> > draw_circles;
    std::vector<std::vector<SquareToDraw> > draw_squares;
    std::vector<std::vector<std::vector<LineToDraw> > > draw_lines;
    draw_lines.resize(num_vertices_);
    draw_squares.resize(num_vertices_);
    for (int i = 0; i < num_vertices_; ++i) {
      draw_lines[i].resize(num_vertices_);
    }

    for (int r = 0; r < state.robots.size(); ++r) {
      const RobotStateIROS14& robot = state.robots[r];
      cv::Scalar color((r * 23456) % 192, (r * 12345) % 256, 0);
      bool robot_in_use = false;
      int destination = robot.destination;
      for (int j = 0; j < state.in_use_robots.size(); ++j) {
        if (state.in_use_robots[j].robot_id == r) {
          destination = state.in_use_robots[j].destination;
          robot_in_use = true;
          break;
        }
      }
      if (robot_in_use) {
        color = cv::Scalar(255, 0, 0);
      }
      std::vector<size_t>* shortest_path = NULL;
      int shortest_path_start_id;
      cv::Point2f robot_pos; 
      if (robot.precision < 0.5f) {
        robot_pos = 
          (1.0f - robot.precision) * bwi_mapper::getLocationFromGraphId(robot.graph_id, graph_) + 
          (robot.precision) * bwi_mapper::getLocationFromGraphId(robot.other_graph_node, graph_);
        shortest_path = &(shortest_paths_[robot.other_graph_node][destination]);
        shortest_path_start_id = robot.other_graph_node;
        LineToDraw l;
        l.priority = r;
        l.precision = robot.precision;
        l.color = color;
        draw_lines[robot.graph_id][robot.other_graph_node].push_back(l); 
      } else {
        robot_pos = 
          robot.precision * bwi_mapper::getLocationFromGraphId(robot.graph_id, graph_) + 
          (1.0f - robot.precision) * bwi_mapper::getLocationFromGraphId(robot.other_graph_node, graph_);
        shortest_path = &(shortest_paths_[robot.graph_id][destination]);
        shortest_path_start_id = robot.graph_id;
        LineToDraw l;
        l.priority = r;
        l.precision = robot.precision;
        l.color = color;
        draw_lines[robot.other_graph_node][robot.graph_id].push_back(l); 
      }
      if (shortest_path->size() != 0) {
        int current_node = shortest_path_start_id;
        for (int s = 0; s < shortest_path->size(); ++s) {
          int next_node = (*shortest_path)[s];
          LineToDraw l;
          l.priority = r;
          l.precision = 0.0f;
          l.color = color;
          draw_lines[current_node][next_node].push_back(l); 
          current_node = next_node;
        }
      }
      draw_circles.push_back(std::make_pair(robot_pos, color));
      SquareToDraw s;
      s.color = color;
      draw_squares[destination].push_back(s);
    }

    // Draw the person's destination
    // SquareToDraw s;
    // s.color = cv::Scalar(0,0,255);
    // draw_squares[goal_idx_].push_back(s);

    for (int i = 0; i < num_vertices_; ++i) {
      for (int j = 0; j < num_vertices_; ++j) {
        std::vector<LineToDraw>& ijlines = draw_lines[i][j];  
        std::vector<LineToDraw>& jilines = draw_lines[j][i];  

        int ijcounter = 0;
        int jicounter = 0;
        int thickness = 2 + 4 * (ijlines.size() + jilines.size() - 1);
        while (ijcounter < ijlines.size() || jicounter < jilines.size()) {
          LineToDraw* line = NULL;
          int s, e;
          if (ijcounter == ijlines.size()) {
            s = j;
            e = i;
            line = &(jilines[jicounter++]);
          } else if (jicounter == jilines.size()) {
            s = i;
            e = j;
            line = &(ijlines[ijcounter++]);
          } else if (ijlines[ijcounter].priority <= 
              jilines[jicounter].priority) {
            s = i;
            e = j;
            line = &(ijlines[ijcounter++]);
          } else {
            s = j;
            e = i;
            line = &(jilines[jicounter++]);
          }
          // Draw this line
          cv::Point2f start_pos = 
            (1.0f - line->precision) * bwi_mapper::getLocationFromGraphId(s, graph_) + 
            line->precision * bwi_mapper::getLocationFromGraphId(e, graph_);
          cv::line(image,
              start_pos,
              bwi_mapper::getLocationFromGraphId(e, graph_),
              line->color, thickness, CV_AA);
          thickness -= 4;
        }
      }
    }

    for (int i = 0; i < num_vertices_; ++i) {
      int size = 18 + 8 * draw_squares[i].size();
      BOOST_FOREACH(const SquareToDraw& square, draw_squares[i]) {
        bwi_mapper::drawSquareOnGraph(image, graph_, i, square.color, 0, 0,
            size);
        size -= 8;
      }
    }

    for (int i = 0; i < draw_circles.size(); ++i) {
      cv::circle(image, draw_circles[i].first, 9, draw_circles[i].second, -1, CV_AA);
    }

    // Draw the person last
    if (state.precision == 1.0f || frame_rate_ == 0.0f) {
      cv::circle(image, 
          bwi_mapper::getLocationFromGraphId(state.graph_id, graph_),
          12, cv::Scalar(0,0,255), -1, CV_AA);
      bwi_mapper::drawArrowOnGraph(image, graph_, 
          std::make_pair(state.graph_id,
                         getAngleInRadians(state.direction) + M_PI/2),
          map_.info.width, map_.info.height);
    } else {
      cv::Point2f human_pos = 
          (1 - state.precision) * bwi_mapper::getLocationFromGraphId(state.from_graph_node, graph_) + 
          state.precision * bwi_mapper::getLocationFromGraphId(state.graph_id, graph_);
      cv::circle(image, human_pos, 12, cv::Scalar(0,0,255), -1, CV_AA);
    }
    bwi_mapper::drawSquareOnGraph(image, graph_, goal_idx_, cv::Scalar(0,0,255),
        0,0,18,-1);

  }

  void PersonModelIROS14::setFrameVector(boost::shared_ptr<std::vector<StateIROS14> >& frame_vector){
    frame_vector_ = frame_vector;
  }
} /* bwi_guidance */
