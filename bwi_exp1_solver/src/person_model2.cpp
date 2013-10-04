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

  PersonModel2::PersonModel2(const topological_mapper::Graph& graph, const
      nav_msgs::OccupancyGrid& map,  size_t goal_idx, const std::string& file,
      bool allow_robot_current_idx, float visibility_range, unsigned int
      max_robots) : graph_(graph), map_(map), goal_idx_(goal_idx),
  allow_robot_current_idx_(allow_robot_current_idx),
  visibility_range_(visibility_range), max_robots_(max_robots) {

    if (!file.empty()) {
      std::ifstream ifs(file.c_str());
      if (ifs.is_open()) {
        std::cout << "PersonModel: Loading model from file: " << file <<
          std::endl;
        boost::archive::binary_iarchive ia(ifs);
        ia >> *this;
        std::cout << " - Model loaded from file!" << std::endl;
        ifs.close();
        return;
      }
    }

    // Compute Model
    initializeStateSpace();
    initializeActionCache();
    initializeNextStateCache();

    std::string out_file = (file.empty()) ? "model.txt" : file;
    std::cout << "PersonModel: Model computed. Saving to file: " << file <<
      std::endl;
    std::ofstream ofs(out_file.c_str());
    boost::archive::binary_oarchive oa(ofs);
    oa << *this;
    ofs.close();
  }

  bool PersonModel2::isTerminalState(const State& state) const {
    return state.graph_id == goal_idx_;
  }

  void PersonModel2::getStateVector(std::vector<State>& states) {
    states = state_cache_;
  }

  void PersonModel2::getActionsAtState(const State& state, 
      std::vector<Action>& actions) {
    actions = action_cache_[state];
  }

  /** Get the predictions of the MDP model for a given state action */
  void PersonModel2::getTransitionDynamics(const State& state, 
      const Action& action, std::vector<State> &next_states, 
      std::vector<float> &rewards, std::vector<float> &probabilities) {

    next_states.clear();
    rewards.clear();
    probabilities.clear();

    if (isTerminalState(state)) {
      return; // no next states for you!
    }

    getNextStates(state, action, next_states); // copy cost
    probabilities = getTransitionProbabilities(state, action);

    rewards.resize(next_states.size());
    // Compute reward based on euclidean distance between state graph ids
    for (std::vector<State>::const_iterator ns = next_states.begin();
        ns != next_states.end(); ++ns) {
      rewards[ns - next_states.begin()] =
        -topological_mapper::getEuclideanDistance(state.graph_id, ns->graph_id,
            graph_);
    }
  }

  void PersonModel2::computeAdjacentVertices() {
    adjacent_vertices_map_.clear();
    for (int graph_id = 0; graph_id < num_vertices_; ++graph_id) {
      std::vector<size_t> adjacent_vertices;
      topological_mapper::getAdjacentNodes(graph_id, graph_, adjacent_vertices); 
      adjacent_vertices_map_[graph_id] = 
        std::vector<int>(adjacent_vertices.begin(), adjacent_vertices.end());
    }
  }

  void PersonModel2::computeVisibleVertices() {
    visible_vertices_map_.clear();
    for (int graph_id = 0; graph_id < num_vertices_; ++graph_id) {
      std::vector<size_t> visible_vertices;
      topological_mapper::getVisibleNodes(graph_id, graph_, map_,
          visible_vertices, visibility_range_); 
      visible_vertices_map_[graph_id] = 
        std::vector<int>(visible_vertices.begin(), visible_vertices.end());
    }
  }

  void PersonModel2::initializeStateSpace() {

    num_vertices_ = boost::num_vertices(graph_);

    computeAdjacentVertices();
    computeVisibleVertices();

    state_cache_.clear();
    for (int graph_id = 0; graph_id < num_vertices_; ++graph_id) {
      std::vector<int>& adjacent_vertices = adjacent_vertices_map_[graph_id];
      std::vector<int>& visible_vertices = visible_vertices_map_[graph_id];
      for (int direction = 0; direction < NUM_DIRECTIONS; ++direction) {
        for (int robots = 0; robots <= max_robots_; ++robots) {

          State state;
          state.graph_id = graph_id;
          state.direction = direction;
          state.num_robots_left = robots;

          // If all 5 robots are available, then no other robots can be placed
          if (robots == max_robots_) {
            state.robot_direction = NONE;
            state.visible_robot = NONE;
            state_cache_.push_back(state);
            continue;
          }

          // Otherwise, place all other robots
          for (int rd = DIR_UNASSIGNED; rd < (int)adjacent_vertices.size();
              ++rd) {

            if (rd == DIR_UNASSIGNED || rd == NONE) { // insert as is
              state.robot_direction = rd; 
            } else { // resolve to adjacent node
              state.robot_direction = adjacent_vertices[rd];
            }

            for (int vr = NONE; vr < (int)visible_vertices.size(); ++vr) {
              // Don't add current_node as an option to visible_robot
              if (vr >= 0 && visible_vertices[vr] == graph_id) {
                continue; 
              }
              if (vr == NONE) { // insert as is
                state.visible_robot = vr;
              } else { // resolve to visible node
                state.visible_robot = visible_vertices[vr];
              }
              state_cache_.push_back(state);
            }
          }
        }
      }
    }
  }

  void PersonModel2::initializeActionCache() {
    action_cache_.clear();
    for (std::vector<State>::iterator it = state_cache_.begin(); 
        it != state_cache_.end(); ++it) {
      constructActionsAtState(*it, action_cache_[*it]);
    }
  }

  void PersonModel2::constructActionsAtState(const State& state, 
      std::vector<Action>& actions) {

    actions.clear();

    // If a direction has to be assigned in the current state, only one of many
    // DIRECT_PERSON actions can be taken
    if (state.robot_direction == DIR_UNASSIGNED) {
      BOOST_FOREACH(int id, adjacent_vertices_map_[state.graph_id]) {
        actions.push_back(Action(DIRECT_PERSON, id));
      }
      return;
    }
    
    // Otherwise have the DO_NOTHING option
    actions.push_back(Action(DO_NOTHING,0));

    // Check if the system can place robots
    if (state.num_robots_left != 0) {
      BOOST_FOREACH(int id, visible_vertices_map_[state.graph_id]) {
        if (state.graph_id != id) {
          if (state.visible_robot == NONE) {
            actions.push_back(Action(PLACE_ROBOT, id)); 
          }
        } else {
          if (allow_robot_current_idx_ && state.robot_direction == NONE) {
            actions.push_back(Action(PLACE_ROBOT, id)); 
          }
        }
      }
    }

  }

  std::vector<Action>& PersonModel2::getActionsAtState(
      const State& state) {
    return action_cache_[state];
  }

  void PersonModel2::initializeNextStateCache() {

    ns_distribution_cache_.clear();
    BOOST_FOREACH(const State& state, state_cache_) {
      std::vector<Action>& actions = getActionsAtState(state);
      BOOST_FOREACH(const Action& action, actions) {
        constructTransitionProbabilities(state, action, 
            ns_distribution_cache_[state][action]);
      }
    }

  }

  void PersonModel2::getNextStates(const State& state, const Action& action, 
      std::vector<State>& next_states) {

    next_states.clear();

    // getNextStates does not check if this action was indeed allowed at the
    // given state. With an incorrect action, this function will probably lead
    // you to a non existent state.

    if (isTerminalState(state)) {
      return; // no next states
    }

    // First figure out next states for all actions that will end up in a
    // deterministic state transition
    if (action.type == PLACE_ROBOT) {
      State next_state = state;
      next_state.num_robots_left--;
      if (action.graph_id != state.graph_id) {
        next_state.visible_robot = action.graph_id;
        next_states.push_back(next_state);
        return;
      } else {
        next_state.robot_direction = DIR_UNASSIGNED;
        next_states.push_back(next_state);
        return;
      }
    }

    if (action.type == DIRECT_PERSON) {
      State next_state = state;
      next_state.robot_direction = action.graph_id;
      next_states.push_back(next_state);
      return;
    }

    // Implies action.type == DO_NOTHING. The next state will not be
    // deterministic. Compute all possible next states. Each next state graph
    // id willl be a member of adjacent nodes, and the direction will be
    // computed automatically
   
    // Get all adjacent ids the person can transition to
    // Algorithm 1 in paper
    BOOST_FOREACH(int next_node, adjacent_vertices_map_[state.graph_id]) {
      State next_state;
      if (state.visible_robot == NONE) {
        // If no robot was visible in previous state, no robot can be present
        next_state.robot_direction = NONE;
        next_state.visible_robot = NONE;
      } else {
        if (state.visible_robot == next_node) {
          // We moved up to a robot, setup a robot here without an assigned dir
          next_state.robot_direction = DIR_UNASSIGNED;
          next_state.visible_robot = NONE; // no longer tracked 
        } else if (std::find(visible_vertices_map_[next_state.graph_id].begin(),
              visible_vertices_map_[next_state.graph_id].end(),
              state.visible_robot) ==
            visible_vertices_map_[next_state.graph_id].end()) { 
          // The person moved such that a previously visible robot is no 
          // longer visible. Decomission the robot.
          next_state.robot_direction = NONE;
          next_state.visible_robot = NONE;
        } else {
          // The case where the tracked robot is still visible
          next_state.robot_direction = NONE;
          next_state.visible_robot = state.visible_robot;
        }
      }
    
      next_state.direction = computeNextDirection(state.direction,
          state.graph_id, next_node, graph_);
      next_state.num_robots_left = state.num_robots_left;
      next_state.graph_id = next_node;
      next_states.push_back(next_state);
    }
  }

  void PersonModel2::constructTransitionProbabilities(const State& state, 
      const Action& action, std::vector<float>& probabilities) {

    probabilities.clear();

    // getNextStates does not check if this action was indeed allowed at the
    // given state. With an incorrect action, this function will probably lead
    // you to a non existent state.

    if (isTerminalState(state)) {
      return; // since next_states.size == 0
    }

    if (action.type == DIRECT_PERSON || action.type == PLACE_ROBOT) {
      probabilities.push_back(1.0f); //since next_states.size == 1
      return;
    }

    // If the person is going to make the transition here (i.e action = DO_NOTHING),
    // then use the hand-coded human motion model for producing the transition
    // function

    // In the future this human motion model needs to be abstracted to an 
    // independent data structure so that it can be learned given a state 
    // feature vector.

    float expected_direction, sigma_sq, random_probability;
    if (state.robot_direction != NONE && 
        state.robot_direction != DIR_UNASSIGNED // shouldn't really happen - VI messed up
        ) {
      expected_direction = topological_mapper::getNodeAngle(state.graph_id,
          state.robot_direction, graph_);
      sigma_sq = 0.1;
      random_probability = 0.1;
    } else {
      expected_direction = 
        getAngleInRadians(state.direction);
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
        graph_location, goal_location, map_);
    int next_identifier_location = state.visible_robot;
    next_identifier_location = 
      (next_identifier_location == NONE && goal_idx_visible) ?
      goal_idx_ : next_identifier_location;
    if (next_identifier_location != NONE) {
      float next_robot_direction = topological_mapper::getNodeAngle(
          state.graph_id, next_identifier_location, graph_);
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
    std::vector<State> next_states;
    getNextStates(state, action, next_states);

    float probability_sum = 0;
    size_t last_non_zero_probability = 0;
    int next_state_counter = 0;
    BOOST_FOREACH(const State& next_state, next_states) {

      float next_state_direction = topological_mapper::getNodeAngle(
          state.graph_id, next_state.graph_id, graph_);

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
      const State& state, const Action& action) {
    return ns_distribution_cache_[state][action];
  }

} /* bwi_exp1 */
