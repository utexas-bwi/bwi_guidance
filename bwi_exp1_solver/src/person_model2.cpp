#include <boost/foreach.hpp>
#include <cmath>
#include <fstream>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

#include <bwi_exp1_solver/person_model2.h>
#include <topological_mapper/point_utils.h>
#include <topological_mapper/map_utils.h>

namespace bwi_exp1 {

  PersonModel2::PersonModel2(const topological_mapper::Graph& graph, 
      const nav_msgs::OccupancyGrid& map,  
      size_t goal_idx, const std::string& file, 
      bool allow_robot_current_idx) : graph_(graph), 
      map_(map), goal_idx_(goal_idx), 
      allow_robot_current_idx_(allow_robot_current_idx) {

    if (!file.empty()) {
      std::ifstream ifs(file.c_str());
      if (ifs.is_open()) {
        std::cout << "Loading model from file: " << file << std::endl;
        boost::archive::binary_iarchive ia(ifs);
        ia >> *this;
        std::cout << "Model loaded!" << std::endl;
        return;
      }
    }

    initializeStateSpace();
    initializeActionCache();
    initializeNextStateCache();

    std::ofstream ofs;
    if (file.empty()) {
      ofs.open("model.txt");
    } else {
      ofs.open(file.c_str());
    }
    boost::archive::binary_oarchive oa(ofs);
    oa << *this;
    /* std::cout << "Saved model to file: " << file << std::endl; */
  }

  bool PersonModel2::isTerminalState(const State2& state) const {
    return state.graph_id == goal_idx_;
  }

  void PersonModel2::getStateVector(std::vector<State2>& states) {
    states = state_cache_;
  }

  void PersonModel2::getActionsAtState(const State2& state, 
      std::vector<Action>& actions) {
    actions = action_cache_[state];
  }

  /** Get the predictions of the MDP model for a given state action */
  void PersonModel2::getTransitionDynamics(const State2& state, 
      const Action& action, std::vector<State2> &next_states, 
      std::vector<float> &rewards, std::vector<float> &probabilities) {

    next_states.clear();
    rewards.clear();
    probabilities.clear();

    if (isTerminalState(state)) {
      return; // no next states for you!
    }

    getNextStates(state, action, next_states); // copy cost
    probabilities = getTransitionProbabilities(state, action);

    if (action.type == DO_NOTHING) {
      rewards.resize(next_states.size());
      // Compute reward
      for (std::vector<State2>::const_iterator ns = next_states.begin();
          ns != next_states.end(); ++ns) {
        rewards[ns - next_states.begin()] = 
          -getEuclideanDistance(state.graph_id, ns->graph_id);
      }
    } else if (action.type == PLACE_ROBOT) {
      // Single transition, no reward
      rewards.push_back(0.0);
    } else {
      // Single transition, no reward
      rewards.push_back(0.0);
    }
  }

  void PersonModel2::computeAdjacentVertices() {
    adjacent_vertices_map_.clear();
    for (int graph_id = 0; graph_id < num_vertices_; ++graph_id) {
      std::vector<size_t> adjacent_vertices;
      topological_mapper::getAdjacentNodes(graph_id, graph_, 
          adjacent_vertices); 
      adjacent_vertices_map_[graph_id] = 
        std::vector<int>(adjacent_vertices.begin(), adjacent_vertices.end());
    }
  }

  void PersonModel2::computeVisibleVertices() {
    visible_vertices_map_.clear();
    for (int graph_id = 0; graph_id < num_vertices_; ++graph_id) {
      topological_mapper::Point2f graph_location =
        topological_mapper::getLocationFromGraphId(graph_id, graph_);
      std::vector<int> &robot_vertices = visible_vertices_map_[graph_id];
      std::set<int> open_set(adjacent_vertices_map_[graph_id].begin(),
          adjacent_vertices_map_[graph_id].end());
      std::set<int> closed_set;
      closed_set.insert(graph_id);
      while (open_set.size() != 0) {
        int graph_id_2 = *(open_set.begin());
        topological_mapper::Point2f graph_location_2 = 
          topological_mapper::getLocationFromGraphId(graph_id_2, graph_); 
        open_set.erase(open_set.begin());
        closed_set.insert(graph_id_2);
        bool location_2_visible = 
          topological_mapper::locationsInDirectLineOfSight(                    
              graph_location, graph_location_2, map_);
        bool location_close = 
          topological_mapper::getMagnitude(graph_location - graph_location_2) <
          (25.0 / map_.info.resolution);
        if (location_2_visible && location_close) {
          // Location is visible from original vertex
          robot_vertices.push_back(graph_id_2);
          // Push all new adjacent vertices into the open set if not in closed
          BOOST_FOREACH(int v, adjacent_vertices_map_[graph_id_2]) {
            if (std::find(closed_set.begin(), closed_set.end(), v) == closed_set.end()) {
              open_set.insert(v);
            }
          }
        }
      }
      
      // Push this vertex into the visible vertices map
      robot_vertices.push_back(graph_id);

      // std::cout << graph_id << " -> ";
      // BOOST_FOREACH(int rv, robot_vertices) {
      //   std::cout << rv << " ";
      // }
      // std::cout << std::endl;
      
    }
  }

  void PersonModel2::initializeStateSpace() {

    num_vertices_ = boost::num_vertices(graph_);
    num_directions_ = 16;
    max_robots_ = 5;

    /* std::cout << "Computing adjacent vertices... " << std::endl; */
    computeAdjacentVertices();
    /* std::cout << "Computing robot vertices... " << std::endl; */
    computeVisibleVertices();

    /* std::cout << "Precaching state space... " << std::endl; */

    state_cache_.clear();
    for (int graph_id = 0; graph_id < num_vertices_; ++graph_id) {
      std::vector<int>& adjacent_vertices = adjacent_vertices_map_[graph_id];
      std::vector<int>& robot_vertices = visible_vertices_map_[graph_id];
      for (int direction = 0; direction < num_directions_; ++direction) {
        for (int robots = 0; robots <= max_robots_; ++robots) {
          if (robots != max_robots_) {
            for (int rd = DIR_UNASSIGNED; 
                rd < (int)adjacent_vertices.size(); ++rd) {
              for (int nrl = NO_ROBOT; nrl < (int)robot_vertices.size(); ++nrl) {
                if (nrl >= 0 && robot_vertices[nrl] == graph_id) {
                  continue;
                }
                State2 state; 
                state.graph_id = graph_id;
                state.direction = direction;
                state.num_robots_left = robots;
                if (rd < 0) {
                  state.current_robot_status = rd;
                } else {
                  state.current_robot_status = 
                    adjacent_vertices[rd];
                }
                if (nrl < 0) {
                  state.visible_robot_location = nrl;
                } else {
                  state.visible_robot_location = robot_vertices[nrl];
                }
                state_cache_.push_back(state);
              }
            }
          } else {
            State2 state;
            state.graph_id = graph_id;
            state.direction = direction;
            state.num_robots_left = robots;
            state.current_robot_status = NO_ROBOT;
            state.visible_robot_location = NO_ROBOT;
            state_cache_.push_back(state);
          }
        }
      }
    }
    /* std::cout << "Number of states: " << state_cache_.size() << std::endl; */
  }

  void PersonModel2::initializeActionCache() {
    action_cache_.clear();
    for (std::vector<State2>::iterator it = state_cache_.begin(); 
        it != state_cache_.end(); ++it) {
      constructActionsAtState(*it, action_cache_[*it]);
    }
  }

  void PersonModel2::constructActionsAtState(const State2& state, 
      std::vector<Action>& actions) {

    actions.clear();
    if (state.current_robot_status == DIR_UNASSIGNED) {
      BOOST_FOREACH(int id, adjacent_vertices_map_[state.graph_id]) {
        actions.push_back(Action(DIRECT_PERSON, id));
      }
      return;
    } else {
      actions.push_back(Action(DO_NOTHING,0));
    }

    if (state.num_robots_left != 0) {
      // If state does not have a future robot, allow placing a robot in a future
      // location
      if (state.visible_robot_location == NO_ROBOT) {
        BOOST_FOREACH(int id, visible_vertices_map_[state.graph_id]) {
          if (state.graph_id != id) {
            actions.push_back(Action(PLACE_ROBOT, id)); 
          }
        }
      }

      if (allow_robot_current_idx_ && state.current_robot_status == NO_ROBOT) {
        actions.push_back(Action(PLACE_ROBOT, state.graph_id));
      }
    }
  }

  std::vector<Action>& PersonModel2::getActionsAtState(
      const State2& state) {
    return action_cache_[state];
  }

  void PersonModel2::initializeNextStateCache() {

    /* std::cout << "Initializing next state cache" << std::endl; */

    next_state_cache_.clear();
    next_state_cache_.resize(num_vertices_ * num_directions_);
    for (int i = 0; i < num_vertices_; ++i) {
      for (int j = 0; j < num_directions_; ++j) {
        BOOST_FOREACH(int id, adjacent_vertices_map_[i]) {
          int direction = computeNextDirection(j, i, id);
          next_state_cache_[i * num_directions_ + j].push_back(
              std::make_pair(id, direction));
        }
      }
    }

    /* std::cout << "Next state cache size: " << next_state_cache_.size() << std::endl;  */

    /* std::cout << "Initializing next state distribution cache" << std::endl; */
    
    ns_distribution_cache_.clear();
    BOOST_FOREACH(const State2& state, state_cache_) {
      std::vector<Action>& actions = getActionsAtState(state);
      BOOST_FOREACH(const Action& action, actions) {
        constructTransitionProbabilities(state, action, 
            ns_distribution_cache_[state][action]);
      }
    }

  }

  void PersonModel2::getNextStates(const State2& state, const Action& action, 
      std::vector<State2>& states) {

    states.clear();

    if (isTerminalState(state)) {
      return; // no next states
    }

    if (action.type == DIRECT_PERSON) {
      State2 next_state = state;
      next_state.current_robot_status = action.graph_id;
      states.push_back(next_state);
      return;
    }
    if (action.type == PLACE_ROBOT) {
      if (action.graph_id != state.graph_id) {
        State2 next_state = state;
        next_state.visible_robot_location = action.graph_id;
        next_state.num_robots_left--;
        states.push_back(next_state);
        return;
      } else {
        State2 next_state = state;
        next_state.current_robot_status = DIR_UNASSIGNED;
        next_state.num_robots_left--;
        states.push_back(next_state);
        return;
      }
    }
   
    // If the person is actually planning on moving, this one is going to be fun 
    // Get all adjacent ids and the directions if we move to those ids
    std::vector<std::pair<int, int> >& next_states = 
      next_state_cache_[state.graph_id * num_directions_ + state.direction];
    typedef std::pair<int, int> int2pair;
    BOOST_FOREACH(int2pair& next_loc, next_states) {
      State2 next_state;
      next_state.graph_id = next_loc.first;
      next_state.direction = next_loc.second;
      next_state.num_robots_left = state.num_robots_left;

      if (state.visible_robot_location == NO_ROBOT) {
        next_state.visible_robot_location = NO_ROBOT;
        next_state.current_robot_status = NO_ROBOT;
      } else {
        // See if the location we moved to is where the next robot is
        if (next_state.graph_id == state.visible_robot_location) {
          // Transition to the state where the current location has a robot, but
          // no direction
          next_state.current_robot_status = DIR_UNASSIGNED;
          next_state.visible_robot_location = NO_ROBOT; // This robot no longer needs to be tracked
        } else {
          // See if the robot is still visible from the current location - you do not want to move
          // to a state where the robot is not visible
          if (std::find(visible_vertices_map_[next_state.graph_id].begin(),
                visible_vertices_map_[next_state.graph_id].end(), state.visible_robot_location) 
              == visible_vertices_map_[next_state.graph_id].end()) { 
            // The person moved in an unexpected manner, the robot gets decommisioned
            next_state.visible_robot_location = NO_ROBOT;
          } else {
            next_state.visible_robot_location = state.visible_robot_location;
          }
          next_state.current_robot_status = NO_ROBOT;
        }
      }
      states.push_back(next_state);
    }

  }

  void PersonModel2::constructTransitionProbabilities(const State2& state, 
      const Action& action, std::vector<float>& probabilities) {

    probabilities.clear();

    if (isTerminalState(state)) {
      return;
    }

    if (action.type == DIRECT_PERSON || action.type == PLACE_ROBOT) {
      // This is a deterministic single transition
      probabilities.push_back(1.0);
      return;
    }

    // If the person is going to be making the transition here, figure out
    // next state probabilities using the person model (i.e action = DO_NOTHING)
    
    // In this simple MDP formulation, the action should induce a desired 
    // direction for the person to be walking to.
    float expected_direction, sigma_sq, random_probability;
    if (state.current_robot_status != NO_ROBOT && 
        state.current_robot_status != DIR_UNASSIGNED // shouldn't really happen - VI messed up
        ) {
      expected_direction = 
        getNodeAngle(state.graph_id, state.current_robot_status);
      sigma_sq = 0.1;
      random_probability = 0.1;
    } else {
      expected_direction = 
        getAngleFromDirection(state.direction);
      sigma_sq = 0.1;
      random_probability = 0.1;
    }

    // In case the next robot is somewhere in the direction of where the person
    // wants to go, this should significantly increase the probablity of seeing
    // the next robot and moving towards it.
    topological_mapper::Point2f graph_location =
      topological_mapper::getLocationFromGraphId(state.graph_id, graph_);
    topological_mapper::Point2f goal_location =
      topological_mapper::getLocationFromGraphId(goal_idx_, graph_);
    bool goal_idx_visible = topological_mapper::locationsInDirectLineOfSight(
        graph_location,
        goal_location,
        map_);
    int next_identifier_location = state.visible_robot_location;
    next_identifier_location = 
      (next_identifier_location == NO_ROBOT && goal_idx_visible) ?
      goal_idx_ : next_identifier_location;
    if (next_identifier_location != NO_ROBOT) {
      float next_robot_direction = 
        getNodeAngle(state.graph_id, next_identifier_location);
      while (next_robot_direction <= expected_direction - M_PI) {
        next_robot_direction += 2 * M_PI;
      }
      while (next_robot_direction > expected_direction + M_PI) {
        next_robot_direction -= 2 * M_PI;
      }
      float difference = fabs(next_robot_direction - expected_direction);
      if (difference < M_PI / 2) {
        sigma_sq = 0.01;
        expected_direction = next_robot_direction;
        random_probability = 0.01;
      }
    }

    // Now compute the weight of each next state. Get the favored direction
    // and compute transition probabilities
    std::vector<State2> next_states;
    getNextStates(state, action, next_states);

    float probability_sum = 0;
    size_t last_non_zero_probability = 0;
    int next_state_counter = 0;
    BOOST_FOREACH(const State2& next_state, next_states) {

      float next_state_direction = 
        getNodeAngle(state.graph_id, next_state.graph_id);

      // wrap next state direction around expected direction
      while (next_state_direction > expected_direction + M_PI) 
        next_state_direction -= 2 * M_PI;
      while (next_state_direction < expected_direction - M_PI) 
        next_state_direction += 2 * M_PI;

      // Compute the probability of this state
      float probability = 
        exp(-pow(next_state_direction-expected_direction, 2) / (2 * sigma_sq));
      probabilities.push_back(probability);
      probability_sum += probability;

      last_non_zero_probability = next_state_counter;
      ++next_state_counter;
    }

    // Normalize probabilities. Ensure sum == 1 with last non zero probability 
    float normalized_probability_sum = 0;
    for (size_t probability_counter = 0; 
        probability_counter < probabilities.size();
        ++probability_counter) {
      probabilities[probability_counter] =
        (1.0 - random_probability)
           * (probabilities[probability_counter] / probability_sum) +
        random_probability * (1.0f / probabilities.size());
      normalized_probability_sum += probabilities[probability_counter];
    }
    probabilities[last_non_zero_probability] += 1 - normalized_probability_sum;

  }

  std::vector<float>& PersonModel2::getTransitionProbabilities(
      const State2& state, const Action& action) {
    return ns_distribution_cache_[state][action];
  }

} /* bwi_exp1 */
