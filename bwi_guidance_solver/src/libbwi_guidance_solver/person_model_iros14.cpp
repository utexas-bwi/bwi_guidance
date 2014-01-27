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

  PersonModelIROS14::PersonModelIROS14(const bwi_mapper::Graph& graph, const
      nav_msgs::OccupancyGrid& map, size_t goal_idx, int
      max_robots_in_use, int action_vertex_visibility_depth, float
      visibility_range, bool allow_goal_visibility, float human_speed, float
      robot_speed) : graph_(graph),
  map_(map), goal_idx_(goal_idx), max_robots_in_use_(max_robots_in_use),
  allow_goal_visibility_(allow_goal_visibility), human_speed_(human_speed),
  robot_speed_(robot_speed), initialized_(false) {

    robot_speed_ /= map_.info.resolution;
    human_speed_ /= map_.info.resolution;

    num_vertices_ = boost::num_vertices(graph_);
    computeAdjacentVertices(adjacent_vertices_map_, graph_);
    computeVisibleVertices(visible_vertices_map_, graph_, map_, visibility_range);

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

    // std::cout << "Action Vertices Map:" << std::endl;
    // for (int i = 0; i < num_vertices_; ++i) {
    //   std::cout << " For vtx " << i << ": ";
    //   BOOST_FOREACH(int vtx, action_vertices_map_[i]) {
    //     std::cout << vtx << " ";
    //   }
    //   std::cout << std::endl;
    // }

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
    if (isRobotDirectionAvailable(dir)) {
      if (dir == DIR_UNASSIGNED) {
        BOOST_FOREACH(int adj, adjacent_vertices_map_[current_state_.graph_id]) {
          actions.push_back(ActionIROS14(GUIDE_PERSON, current_state_.graph_id, adj));
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

  bool PersonModelIROS14::isRobotDirectionAvailable(int& robot_dir) {
    // Figure out if there is a robot at the current position
    for (int i = 0; i < current_state_.in_use_robots.size(); ++i) {
      int destination = current_state_.in_use_robots[i].destination;
      int direction = current_state_.in_use_robots[i].direction;
      int robot_graph_id = current_state_.robots[current_state_.in_use_robots[i].robot_id].graph_id;
      float robot_precision = current_state_.robots[current_state_.in_use_robots[i].robot_id].precision;
      if (destination != current_state_.graph_id ||
          robot_graph_id != current_state_.graph_id ||
          robot_precision < 0.0f)
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

  void PersonModelIROS14::moveRobots(float time) {
    /* std::cout << "Moving ahead for " << time << " seconds" << std::endl; */
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
            robot.destination = generateNewGoalFrom(ROBOT_HOME_BASE[i]);
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
        if (action.at_graph_id == current_state_.in_use_robots[i].destination) {
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
      return 0.0;
    }

    if (action.type == ASSIGN_ROBOT) {
      assert(current_state_.in_use_robots.size() < max_robots_in_use_);
      float distance_to_destination = shortest_distances_[current_state_.graph_id][action.at_graph_id];
      float time_to_destination = distance_to_destination / human_speed_; 
      InUseRobotStateIROS14 r;
      r.robot_id = 
        selectBestRobotForTask(action.at_graph_id, time_to_destination);
      r.destination = action.at_graph_id;
      r.direction = DIR_UNASSIGNED;
      current_state_.in_use_robots.push_back(r);
      current_state_.acquired_locations.push_back(action.at_graph_id);
      return -20.0;
    }

    if (action.type == GUIDE_PERSON) {
      int mark = -1;
      for (int i = 0; i < current_state_.in_use_robots.size(); ++i) {
        if (action.at_graph_id == current_state_.in_use_robots[i].destination) {
          mark = i;
          break;
        }
      }
      assert(mark != -1);
      current_state_.in_use_robots[mark].direction = action.guide_graph_id;
    }

    // alright, need to wait for the human to take an action - let's first
    // figure out what action he takes
    float expected_dir = getAngleInRadians(current_state_.direction);
    int robot_dir = 0;
    if (isRobotDirectionAvailable(robot_dir)) {
      assert(robot_dir != DIR_UNASSIGNED);
      if (robot_dir != DIR_UNASSIGNED) {
        expected_dir = bwi_mapper::getNodeAngle(
          current_state_.graph_id, robot_dir, graph_);
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
      float probability = 0.9 * (weights[probability_counter] / weight_sum) +
        0.1 * (1.0f / weights.size());
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
    current_state_.graph_id = next_node;
    current_state_.acquired_locations.clear();
    current_state_.relieved_locations.clear();

    // TODO allow moving robots and people slowly for visualization
    moveRobots(time_to_vertex);

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
      StateIROS14 &state, bool &terminal) {

    //boost::posix_time::ptime mst1 = boost::posix_time::microsec_clock::local_time();

    assert(initialized_);
    assert(ugen_);
    assert(!isTerminalState(current_state_));

    reward = takeActionAtCurrentState(action);
    state = current_state_;
    terminal = isTerminalState(current_state_);

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
      state.robots.push_back(robot);
    }
  }

  void PersonModelIROS14::initializeRNG(UIGenPtr uigen, URGenPtr ugen, 
      PIGenPtr pgen) {
    uigen_ = uigen;
    ugen_ = ugen;
    pgen_ = pgen;
  }

  void PersonModelIROS14::drawCurrentState(cv::Mat& image) {
    assert(initialized_);

    bwi_mapper::drawCircleOnGraph(image, graph_, current_state_.graph_id);
    bwi_mapper::drawArrowOnGraph(image, graph_, 
        std::make_pair(current_state_.graph_id,
                       getAngleInRadians(current_state_.direction) + M_PI/2),
        map_.info.width, map_.info.height);

    for (int r = 0; r < current_state_.robots.size(); ++r) {
      RobotStateIROS14& robot = current_state_.robots[r];
      cv::Scalar color((r * 12345) % 192,128, (r * 23456) % 192);
      bool robot_in_use = false;
      int destination = robot.destination;
      for (int j = 0; j < current_state_.in_use_robots.size(); ++j) {
        if (current_state_.in_use_robots[j].robot_id == r) {
          destination = current_state_.in_use_robots[j].destination;
          robot_in_use = true;
          break;
        }
      }
      if (robot_in_use) {
        color = cv::Scalar(128, 0, 128);
      }
      
      cv::Point2f robot_pos;
      bool draw_first_link;
      std::vector<size_t>& shortest_path = shortest_paths_[robot.graph_id][destination];
      if (robot.precision < 0.0f) {
        robot_pos = 
          -robot.precision * bwi_mapper::getLocationFromGraphId(robot.from_graph_node, graph_) + 
          (1 + robot.precision) * bwi_mapper::getLocationFromGraphId(robot.graph_id, graph_);
        cv::line(image,
            bwi_mapper::getLocationFromGraphId(robot.graph_id, graph_),
            robot_pos, color, 2);
        draw_first_link = true;
      } else {
        int next_node = robot.graph_id;
        if (shortest_path.size() > 0) {
          next_node = shortest_path[0];
        }
        robot_pos = 
          (1 - robot.precision) * bwi_mapper::getLocationFromGraphId(robot.graph_id, graph_) + 
          (robot.precision) * bwi_mapper::getLocationFromGraphId(next_node, graph_);
        cv::line(image,
            bwi_mapper::getLocationFromGraphId(next_node, graph_),
            robot_pos, color, 2);
        draw_first_link = false;
      }
      int current_node = robot.graph_id;
      for (int s = 0; s < shortest_path.size(); ++s) {
        int next_node = shortest_path[s];
        if (s != 0 || draw_first_link) {
          cv::line(image,
              bwi_mapper::getLocationFromGraphId(current_node, graph_),
              bwi_mapper::getLocationFromGraphId(next_node, graph_),
              color, 2);
        }
        current_node = next_node;
      }
      cv::circle(image, robot_pos, 10, color, -1);
      bwi_mapper::drawSquareOnGraph(image, graph_, destination, color);
    }
  }

  void PersonModelIROS14::printDistanceToDestination(int idx) {
    assert(idx >= 0 && idx < current_state_.robots.size());
    std::cout << "Distance of Robot " << idx << " to its destination: " <<
      getTrueDistanceTo(current_state_.robots[idx],
          current_state_.robots[idx].destination) << std::endl;
  }
} /* bwi_guidance */
