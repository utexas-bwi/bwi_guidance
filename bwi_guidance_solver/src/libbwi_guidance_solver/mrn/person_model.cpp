#include <boost/foreach.hpp>
#include <cassert>
#include <cmath>
#include <fstream>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

#include <bwi_guidance_solver/mrn/person_model.h>
#include <bwi_mapper/point_utils.h>
#include <bwi_mapper/map_utils.h>

namespace bwi_guidance_solver {

  namespace mrn {
    
    PersonModel::PersonModel(const bwi_mapper::Graph& graph, 
                             const nav_msgs::OccupancyGrid& map, size_t goal_idx, float frame_rate,
                             int max_robots_in_use, int action_vertex_visibility_depth, 
                             int action_vertex_adjacency_depth, float visibility_range, 
                             float human_speed, float robot_speed,
                             float utility_multiplier, bool use_shaping_reward, 
                             bool discourage_bad_assignments) :
      graph_(graph), map_(map), goal_idx_(goal_idx),
      frame_rate_(frame_rate), max_robots_in_use_(max_robots_in_use),
      human_speed_(human_speed),
      robot_speed_(robot_speed), utility_multiplier_(utility_multiplier),
      use_shaping_reward_(use_shaping_reward),
      discourage_bad_assignments_(discourage_bad_assignments) {

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

    bool PersonModel::isTerminalState(const State& state) const {
      return state.graph_id == goal_idx_;
    }

    void PersonModel::getActionsAtState(const State& state, 
                                        std::vector<Action>& actions) {
      actions.clear();
      if (isAssignedRobotColocated(state)) {
        actions.resize(2 * adjacent_vertices_map_[state.graph_id].size());
        for (unsigned int i = 0; 
             i < adjacent_vertices_map_[state.graph_id].size(); ++i) {
          actions[2 * i] = Action(GUIDE_PERSON, state.graph_id, adjacent_vertices_map_[state.graph_id][i]);
          actions[(2 * i) + 1] = Action(LEAD_PERSON, state.graph_id, adjacent_vertices_map_[state.graph_id][i]);

        }
        return; // Only choose a direction here
      }
      actions.push_back(Action(WAIT, 0, 0));
      std::vector<int> cant_assign_vertices = state.relieved_locations;
      for (int i = 0; i < state.in_use_robots.size(); ++i) {
        if (std::find(state.acquired_locations.begin(),
                      state.acquired_locations.end(),
                      state.in_use_robots[i].destination) ==
            state.acquired_locations.end()) {
          actions.push_back(Action(RELEASE_ROBOT, 
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
            actions.push_back(Action(ASSIGN_ROBOT, vtx, DIR_UNASSIGNED));
          }
        }
      }
    }

    float PersonModel::getTrueDistanceTo(RobotState& robot, 
                                         int current_destination, 
                                         int to_destination, bool change_robot_state) {

      // Optimized!!!
      float ret_distance;
      float current_edge_distance = shortest_distances_[robot.graph_id][robot.other_graph_node];
      if (robot.precision < 0.5f) {
        // We are going from graph_id to other_graph_node.
        float distance_through_current_node = 
          robot.precision * current_edge_distance + shortest_distances_[robot.graph_id][to_destination];
        float distance_through_other_node = 
          (1.0f - robot.precision) * current_edge_distance
          + shortest_distances_[robot.other_graph_node][to_destination];
        if (distance_through_current_node < distance_through_other_node) {
          ret_distance = distance_through_current_node;
          // We need to backtrack and the robot needs to be flipped around.
          if (change_robot_state) {
            if (robot.precision != 0.0f) {
              robot.precision = 1.0f - robot.precision;
            } else {
              // The robot is exactly at robot.graph_id, find shortest path to destination
              std::vector<size_t>& shortest_path = shortest_paths_[robot.graph_id][to_destination];
              if (shortest_path.size() > 0) {
                robot.other_graph_node = shortest_path[0];
              } else /* The robot is at the destination */ {
                robot.other_graph_node = robot.graph_id;
              }
            }
          }
        } else {
          ret_distance = distance_through_other_node;
          // Since the optimal path goes through the other node, we have to approach it. Change nothing.
        }
      } else {
        // We are going from other_graph_node to graph_id.
        float distance_through_current_node = 
          (1.0f - robot.precision) * current_edge_distance + shortest_distances_[robot.graph_id][to_destination];
        float distance_through_other_node = 
          robot.precision * current_edge_distance + shortest_distances_[robot.other_graph_node][to_destination];
        if (distance_through_current_node <= distance_through_other_node) {
          ret_distance = distance_through_current_node;
          // Nothing needs to change for optimal solution
        } else {
          ret_distance = distance_through_other_node;
          // The robot should be flipped around
          if (change_robot_state) {
            // The destination should have changed externally already
            robot.precision = 1.0f - robot.precision;
          }
        }
      }
      return ret_distance;
    }

    void PersonModel::changeRobotDirectionIfNeeded(RobotState& state, 
                                                   int current_destination, int to_destination) {
      // This will flip the robot around based on the optimal path
      getTrueDistanceTo(state, current_destination, to_destination, true);
    }

    int PersonModel::selectBestRobotForTask(const State& state, int destination, 
                                            float time_to_destination, bool& reach_in_time) {

      std::vector<float> utility_loss_all(state.robots.size(), std::numeric_limits<float>::max());
      std::vector<float> utility_loss_in_time(state.robots.size(), std::numeric_limits<float>::max());
      std::vector<float> time(state.robots.size(), std::numeric_limits<float>::max());

      for (int i = 0; i < state.robots.size(); ++i) {
        bool robot_in_use = false;
        for (int j = 0; j < state.in_use_robots.size(); ++j) {
          if (state.in_use_robots[j].robot_id == i) {
            robot_in_use = true;
            break;
          }
        }
        if (!robot_in_use) {
          // We don't need to change the state, but we need to provide a mutable state to getTrueDistanceTo. Hence the
          // mutable copy. Probably bad design!.
          State mutable_state = state;
          float orig_distance = getTrueDistanceTo(mutable_state.robots[i],
                                                  state.robots[i].destination, 
                                                  state.robots[i].destination);
          float original_time = orig_distance / robot_speed_;
          float new_distance_1 = getTrueDistanceTo(mutable_state.robots[i],
                                                   state.robots[i].destination,
                                                   destination);
          float new_distance_2 = shortest_distances_[destination][state.robots[i].destination];
          float new_time = 
            std::max(new_distance_1 / robot_speed_, time_to_destination) + 
            new_distance_2 / robot_speed_;
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
        reach_in_time = true;
        return std::min_element(utility_loss_in_time.begin(), utility_loss_in_time.end()) - 
          utility_loss_in_time.begin();
      } else {
        reach_in_time = false;
        return std::min_element(time.begin(), time.end()) - time.begin();
      }

    }

    bool PersonModel::isAssignedRobotColocated(const State& state) {
      // Figure out if there is a robot at the current position
      for (int i = 0; i < state.in_use_robots.size(); ++i) {
        int destination = state.in_use_robots[i].destination;
        int robot_graph_id = state.robots[state.in_use_robots[i].robot_id].graph_id;
        bool reached_destination = state.in_use_robots[i].reached_destination;
        if (destination == state.graph_id &&
            robot_graph_id != state.graph_id &&
            reached_destination) {
          return true;
        }
      }
      return false;
    }

    void PersonModel::cacheShortestPaths() {
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

            shortest_distances_[idx][j] = 
              bwi_mapper::getShortestPathWithDistance(idx, j, shortest_paths_[idx][j], graph_);

            // Post-process the shortest path - add goal, remove start and reverse
            shortest_paths_[idx][j].insert(shortest_paths_[idx][j].begin(), j); // Add j
            shortest_paths_[idx][j].pop_back(); // Remove idx
            std::reverse(shortest_paths_[idx][j].begin(), shortest_paths_[idx][j].end());
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

    void PersonModel::cacheNewGoalsByDistance() {
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

    int PersonModel::generateNewGoalFrom(int idx, boost::shared_ptr<RNG> &rng) {
      // Optimized!!!
      assert(goals_by_distance_.size() == num_vertices_);
      while(true) {
        int graph_distance = rng->poissonInt(1);
        if (graph_distance >= goals_by_distance_[idx].size()) {
          continue;
        }
        std::vector<int>& possible_goals = goals_by_distance_[idx][graph_distance];
        return *(possible_goals.begin() + rng->randomInt(possible_goals.size() - 1));
      }
    }

    bool PersonModel::moveRobots(State& state, float time, boost::shared_ptr<RNG> &rng, float human_speed) {
      // Move the human
      bool ready_for_next_action = true;
      if (frame_rate_ > 0.0f) {
        float human_coverable_distance = time * human_speed;
        float human_edge_distance = shortest_distances_[state.from_graph_node][state.graph_id];
        float added_precision = human_coverable_distance / human_edge_distance;
        state.precision += added_precision;
        if (state.precision < 1.0f) {
          ready_for_next_action = false;
        } else {
          time -= (state.precision - 1.0f) * human_edge_distance / human_speed;
          state.precision = 1.0f;
        }
      }

      /* std::cout << "Moving ahead for " << time << " seconds" << std::endl; */
      // Optimized!!!
      for (int i = 0; i < state.robots.size(); ++i) {
        RobotState& robot = state.robots[i];

        // Check if we are moving this robot under our control or not
        int destination = robot.destination;
        bool robot_in_use = false;
        bool* robot_reached = NULL;
        for (int j = 0; j < state.in_use_robots.size(); ++j) {
          if (state.in_use_robots[j].robot_id == i) {
            destination = state.in_use_robots[j].destination;
            robot_reached = &(state.in_use_robots[j].reached_destination);
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
              robot.destination = generateNewGoalFrom(ROBOT_HOME_BASE[i], rng);
              destination = robot.destination;
              std::vector<size_t>& shortest_path = shortest_paths_[robot.graph_id][robot.destination];
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
            // if (current_edge_distance == 0.0f) {
            //   std::cout << robot.graph_id << " " << robot.precision << " " << destination << " " << robot.other_graph_node << std::endl;
            // }
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

    void PersonModel::takeAction(const State &state, 
                                 const Action &action, 
                                 float &reward, 
                                 State &next_state, 
                                 bool &terminal, 
                                 int &depth_count,
                                 boost::shared_ptr<RNG> &rng,
                                 float &time_loss,
                                 float &utility_loss,
                                 std::vector<State> &frame_vector) {
      next_state = state;
      reward = 0.0f;

      if (action.type == RELEASE_ROBOT) {
        int mark_for_removal = -1;
        int robot_id;
        for (int i = 0; i < next_state.in_use_robots.size(); ++i) {
          if (action.at_graph_id == next_state.in_use_robots[i].destination) {
            robot_id = next_state.in_use_robots[i].robot_id;
            mark_for_removal = i;
            break;
          }
        }
        assert(mark_for_removal != -1);
        next_state.in_use_robots.erase(next_state.in_use_robots.begin() + mark_for_removal);
        next_state.relieved_locations.push_back(action.at_graph_id);

        // Since we are changing destinations, we need to reset robot state
        // to take optimal path to goal
        RobotState& robot = next_state.robots[robot_id];
        changeRobotDirectionIfNeeded(robot, action.at_graph_id, robot.destination);
        utility_loss = 0.0f;
        time_loss = 0.0f;
        reward = 0.0f;
      } else if (action.type == ASSIGN_ROBOT) {
        /* std::cout << next_state << std::endl; */
        assert(next_state.in_use_robots.size() < max_robots_in_use_);
        float distance_to_destination = shortest_distances_[next_state.graph_id][action.at_graph_id];
        float time_to_destination = distance_to_destination / human_speed_; 
        InUseRobotState r;
        bool reach_in_time;
        r.robot_id = selectBestRobotForTask(next_state, action.at_graph_id, time_to_destination, reach_in_time);
        r.destination = action.at_graph_id;
        r.reached_destination = 
          next_state.robots[r.robot_id].graph_id == r.destination &&
          next_state.robots[r.robot_id].precision == 0.0f;
        next_state.in_use_robots.push_back(r);
        next_state.acquired_locations.push_back(action.at_graph_id);

        RobotState& robot = next_state.robots[r.robot_id];
        changeRobotDirectionIfNeeded(robot, robot.destination, action.at_graph_id);

        /* std::cout << next_state << std::endl; */
        utility_loss = 0.0f;
        time_loss = 0.0f;
        if (discourage_bad_assignments_ && !reach_in_time) {
          reward = -50.0f;
        } else {
          reward = 0.0f;
        }
      } else if (action.type == GUIDE_PERSON) {
        int mark = -1;
        int robot_id;
        for (int i = 0; i < next_state.in_use_robots.size(); ++i) {
          if (action.at_graph_id == next_state.in_use_robots[i].destination) {
            robot_id = next_state.in_use_robots[i].robot_id;
            mark = i;
            break;
          }
        }
        assert(mark != -1);
        //next_state.in_use_robots[mark].direction = action.guide_graph_id;
        float direction = bwi_mapper::getNodeAngle(next_state.graph_id, action.guide_graph_id, graph_);
        next_state.direction = getDiscretizedAngle(direction);
        next_state.robot_gave_direction = true;
        next_state.in_use_robots.erase(next_state.in_use_robots.begin() + mark);
        next_state.relieved_locations.push_back(action.at_graph_id);

        // Since we are changing destinations, we need to reset robot state
        // to take optimal path to goal
        RobotState& robot = next_state.robots[robot_id];
        changeRobotDirectionIfNeeded(robot, action.at_graph_id, robot.destination);
        utility_loss = 0.0f;
        time_loss = 0.0f;
        reward = 0.0f;
      } else /* (action.type == LEAD_PERSON) || (action.type == WAIT) */ {

        // alright, need to wait for the human to take an action - let's first
        // figure out what action he takes
        int next_node;
        float human_speed;
        if (action.type == WAIT) {

          float expected_dir = getAngleInRadians(next_state.direction);
          double sigma_sq = 0.1;
          if (next_state.robot_gave_direction) {
            sigma_sq = 0.05;
          }

          // Now assume that the person moves to one the adjacent locations
          double weight_sum = 0;
          std::vector<double> weights;
          BOOST_FOREACH(int adj, adjacent_vertices_map_[next_state.graph_id]) {

            float next_state_direction = bwi_mapper::getNodeAngle(next_state.graph_id, adj, graph_);
            float angle_difference = getAbsoluteAngleDifference(next_state_direction, expected_dir);

            // Compute the probability of this state
            double weight = exp(-pow(angle_difference, 2) / (2 * sigma_sq));
            weights.push_back(weight);
            weight_sum += weight;
          }

          float probability_sum = 0;
          std::vector<float> probabilities;
          /* std::cout << "Transition probabilities: " << std::endl; */
          for (size_t probability_counter = 0; probability_counter < weights.size();
               ++probability_counter) {
            /* std::cout << weights[probability_counter] << " " << weight_sum << std::endl; */
            double probability = 0.99 * (weights[probability_counter] / weight_sum) +
              0.01 * (1.0f / weights.size());
            probability_sum += probability;
            if (probability_counter == weights.size() - 1) {
              // Account for floating point errors. No surprises!
              probability += 1.0f - probability_sum; 
            }
            probabilities.push_back(probability);
            // std::cout << "  to " << 
            //   adjacent_vertices_map_[next_state.graph_id][probability_counter] <<
            //   ": " << probability << std::endl;
          }

          next_node = adjacent_vertices_map_[next_state.graph_id][rng->select(probabilities)];
          human_speed = human_speed_;

          // Now that we've decided which vertex the person is moving to, compute
          // time to that vertex and update all the robots
        } else {
          next_node = action.guide_graph_id;
          human_speed = robot_speed_;

          // Also update the locate of the robot in use.
          int mark = -1;
          int robot_id;
          int in_use_robot_id;
          for (int i = 0; i < next_state.in_use_robots.size(); ++i) {
            if (action.at_graph_id == next_state.in_use_robots[i].destination) {
              robot_id = next_state.in_use_robots[i].robot_id;
              in_use_robot_id = i;
              mark = i;
              break;
            }
          }
          assert(mark != -1);

          // Now that we've found the robot, move the robot also.
          RobotState& robot = next_state.robots[robot_id];
          robot.other_graph_node = action.guide_graph_id;
          InUseRobotState& in_use_robot = next_state.in_use_robots[in_use_robot_id];
          in_use_robot.destination = action.guide_graph_id;

        }

        float time_to_vertex = shortest_distances_[next_state.graph_id][next_node] / human_speed;

        std::vector<float> time_to_original_destination_before(10);
        BOOST_FOREACH(const InUseRobotState& robot, next_state.in_use_robots) {
          float distance_to_original_destination = 
            getTrueDistanceTo(next_state.robots[robot.robot_id], 0, 
                              next_state.robots[robot.robot_id].destination);
          time_to_original_destination_before[robot.robot_id] = 
            distance_to_original_destination / robot_speed_;
        }

        // Transition to next state
        float distance_closed = 
          shortest_distances_[next_state.graph_id][goal_idx_] -
          shortest_distances_[next_node][goal_idx_];

        next_state.direction = computeNextDirection(next_state.direction, next_state.graph_id, next_node, graph_);
        next_state.robot_gave_direction = false;
        next_state.precision = 0.0f;
        next_state.from_graph_node = next_state.graph_id;
        next_state.graph_id = next_node;
        next_state.acquired_locations.clear();
        next_state.relieved_locations.clear();

        if (frame_rate_ == 0.0f) {
          moveRobots(next_state, time_to_vertex, rng, human_speed);
        } else {
          frame_vector.clear();
          while (!moveRobots(next_state, 1.0f / frame_rate_, rng, human_speed)) {
            frame_vector.push_back(next_state);
          }
        }

        // After the robots have been moved, let's compute the time to destination
        // again to compute the utility loss
        utility_loss = 0.0f;
        BOOST_FOREACH(const InUseRobotState& robot, 
                      next_state.in_use_robots) {
          float distance_to_original_destination = 
            getTrueDistanceTo(next_state.robots[robot.robot_id], 0, 
                              next_state.robots[robot.robot_id].destination);
          float time_to_original_destination = 
            distance_to_original_destination / robot_speed_;
          utility_loss += 
            std::max(0.0f, 
                     time_to_original_destination + time_to_vertex -
                     time_to_original_destination_before[robot.robot_id]);
        }
        time_loss = time_to_vertex;

        // Compute reward
        reward = -time_to_vertex;

        if (use_shaping_reward_) {
          reward += distance_closed / human_speed_;
        }

        reward -= utility_multiplier_ * utility_loss; 
      }
      
      terminal = isTerminalState(next_state);
      depth_count = (action.type != WAIT) ? 0 : lrint(time_loss); 
    }

    void PersonModel::takeAction(const State &state, const Action &action, float &reward, 
                                 State &next_state, bool &terminal, int &depth_count, boost::shared_ptr<RNG> rng) {

      float unused_utility_loss, unused_time_loss;
      std::vector<State> unused_frame_vector;
      takeAction(state, action, reward, next_state, terminal, depth_count, rng,
                 unused_time_loss, unused_utility_loss, unused_frame_vector);
    }

    void PersonModel::getFirstAction(const State &state, 
                                     Action &action) {
      // Optimized!
      get_action_state_ = state;
      getActionsAtState(state, get_actions_);
      action = get_actions_[0];
      get_actions_counter_ = 1;
    }

    bool PersonModel::getNextAction(const State &state, 
                                    Action &action) {
      assert(state == get_action_state_);
      if (get_actions_counter_ < get_actions_.size()) {
        action = get_actions_[get_actions_counter_++];
        return true;
      }
      return false;
      // bool return_next = action.type == WAIT;
      // std::vector<int> assigned_vertices(state.in_use_robots.size());
      // for (int i = 0; i < state.in_use_robots.size(); ++i) {
      //   Action next_action(RELEASE_ROBOT, 
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
      //         Action next_action(ASSIGN_ROBOT, vtx, adj);
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

    void PersonModel::getAllActions(const State &state,
                                    std::vector<Action>& actions) {
      getActionsAtState(state, actions);
    }

    void PersonModel::addRobots(State& state, int n, boost::shared_ptr<RNG> &rng) {
      for (int r = 0; r < n; ++r) {
        RobotState robot;
        robot.graph_id = ROBOT_HOME_BASE[r];
        robot.destination = generateNewGoalFrom(robot.graph_id, rng); 
        robot.precision = 0.0f;
        if (shortest_paths_[robot.graph_id][robot.destination].size() != 0) {
          robot.other_graph_node = shortest_paths_[robot.graph_id][robot.destination][0];
        } else {
          robot.other_graph_node = robot.graph_id;
        }
        state.robots.push_back(robot);
      }
    }

    void PersonModel::drawState(const State& state, cv::Mat& image) {

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
        cv::circle(image, human_pos, 15, cv::Scalar(0,0,255), -1, CV_AA);
      }

      std::vector<std::pair<cv::Point2f, cv::Scalar> > draw_circles;
      std::vector<std::vector<SquareToDraw> > draw_squares;
      std::vector<std::vector<std::vector<LineToDraw> > > draw_lines;
      draw_lines.resize(num_vertices_);
      draw_squares.resize(num_vertices_);
      for (int i = 0; i < num_vertices_; ++i) {
        draw_lines[i].resize(num_vertices_);
      }

      for (int r = 0; r < state.robots.size(); ++r) {
        RobotState robot = state.robots[r];
        cv::Scalar color(0, (r * 12345) % 256, 0);
        bool robot_in_use = false;
        std::vector<int> destinations;
        destinations.push_back(robot.destination);
        for (int j = 0; j < state.in_use_robots.size(); ++j) {
          if (state.in_use_robots[j].robot_id == r) {
            destinations.insert(destinations.begin(), state.in_use_robots[j].destination);
            robot_in_use = true;
            break;
          }
        }
        if (robot_in_use) {
          color = cv::Scalar(255, 0, 0);
        }
        bool use_dashed_line = false;
        BOOST_FOREACH(int destination, destinations) {
          changeRobotDirectionIfNeeded(robot, 0, destination);
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
            l.dashed = use_dashed_line;
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
            l.dashed = use_dashed_line;
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
              l.dashed = use_dashed_line;
              draw_lines[current_node][next_node].push_back(l); 
              current_node = next_node;
            }
          }
          draw_circles.push_back(std::make_pair(robot_pos, color));
          SquareToDraw s;
          s.color = color;
          draw_squares[destination].push_back(s);
          use_dashed_line = true;
        }
      }

      // Draw the person's destination
      // SquareToDraw s;
      // s.color = cv::Scalar(0,0,255);
      // draw_squares[goal_idx_].push_back(s);

      for (int i = 0; i < num_vertices_; ++i) {
        for (int j = i+1; j < num_vertices_; ++j) {
          std::vector<LineToDraw>& ijlines = draw_lines[i][j];  
          std::vector<LineToDraw>& jilines = draw_lines[j][i];  

          // For each line, store a standard vector of thickness based on priority
          std::set<int> priorities;
          BOOST_FOREACH(const LineToDraw& l, ijlines) {
            priorities.insert(l.priority);
          }
          BOOST_FOREACH(const LineToDraw& l, jilines) {
            priorities.insert(l.priority);
          }

          std::map<int, int> thickness_map;
          int p_count = priorities.size() - 1;
          BOOST_FOREACH(int p, priorities) {
            thickness_map[p] = 2 + 4 * p_count;
            --p_count;
          }

          int ijcounter = 0;
          int jicounter = 0;
          // if (thickness != 2) {
          //   std::cout << "ST: " << thickness << std::endl;
          // }
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
            if (line->dashed) {
              dashedLine(image, bwi_mapper::getLocationFromGraphId(e, graph_), start_pos, line->color, 10,
                         thickness_map[line->priority], CV_AA);
            } else {
              cv::line(image,
                       bwi_mapper::getLocationFromGraphId(e, graph_),
                       start_pos,
                       line->color, thickness_map[line->priority], CV_AA);
            }
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

      bwi_mapper::drawSquareOnGraph(image, graph_, goal_idx_, cv::Scalar(0,0,255),
                                    0,0,18,-1);

    }

  } /* mrn */

} /* bwi_guidance */
