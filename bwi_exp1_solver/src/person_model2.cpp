#include <boost/foreach.hpp>
#include <cmath>

#include <bwi_exp1_solver/person_model2.h>
#include <topological_mapper/point_utils.h>
#include <topological_mapper/map_utils.h>

namespace bwi_exp1 {

  Action::Action() : type(DO_NOTHING), graph_id(0) {}
  Action::Action(ActionType a, size_t g) : type(a), graph_id(g) {}

  PersonModel2::PersonModel2(const topological_mapper::Graph& graph, 
      size_t goal_idx) : graph_(graph), goal_idx_(goal_idx) {
    initializeStateSpace();
    initializeActionCache();
    initializeNextStateCache();
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
          -getDistanceFromStates(state.graph_id, ns->graph_id);
      }
    } else {
      // Single transition, no reward
      rewards.push_back(0.0);
    }
  }

  void PersonModel2::computeAdjacentVertices() {
    adjacent_vertices_map_.clear();
    for (int graph_id = 0; graph_id < num_vertices_; ++graph_id) {
      std::vector<size_t> adjacent_vertices;
      topological_mapper::getAdjacentVertices(graph_id, graph_, 
          adjacent_vertices); 
      adjacent_vertices_map_[graph_id] = 
        std::vector<int>(adjacent_vertices.begin(), adjacent_vertices.end());
    }
  }

  void PersonModel2::computeRobotVertices() {
    robot_vertices_map_.clear();
    for (int graph_id = 0; graph_id < num_vertices_; ++graph_id) {
      topological_mapper::Point2f graph_location =
        topological_mapper::getLocationFromGraphId(graph_id, graph_);
      std::vector<int> &robot_vertices = robot_vertices_map_[graph_id];
      std::set<int> open_set(adjacent_vertices_map_[graph_id].begin(),
          adjacent_vertices_map_[graph_id].end());
      std::set<int> closed_set;
      closed_set.insert(graph_id);
      while (open_set.size() != 0) {
        int graph_id_2 = *(open_set.begin());
        open_set.erase(open_set.begin());
        closed_set.insert(graph_id_2);
        if (topological_mapper::locationInDirectLineOfSight(
              graph_location, 
              topological_mapper::getLocationFromGraphId(graph_id_2, graph_),
              map_)) {
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
    }
  }

  void PersonModel2::initializeStateSpace() {

    num_vertices_ = boost::num_vertices(graph_);
    num_directions_ = 16;
    max_robots_ = 5;

    computeAdjacentVertices();
    computeRobotVertices();

    state_cache_.clear();
    size_t size = 0;
    for (int graph_id = 0; graph_id < num_vertices_; ++graph_id) {
      std::vector<int>& adjacent_vertices = adjacent_vertices_map_[graph_id];
      std::vector<int>& robot_vertices = robot_vertices_map_[graph_id];
      size += num_directions_ * (max_robots_ + 1) * 
        (adjacent_vertices.size() + 2) * (robot_vertices.size() + 1) *
        (robot_vertices.size() + 1);
    }
    state_cache_.resize(size);

    size_t count = 0;
    for (int graph_id = 0; graph_id < num_vertices_; ++graph_id) {
      std::vector<int>& adjacent_vertices = adjacent_vertices_map_[graph_id];
      std::vector<int>& robot_vertices = robot_vertices_map_[graph_id];
      for (int direction = 0; direction < num_directions_; ++direction) {
        for (int robots = 0; robots <= max_robots_; ++robots) {
          for (int rd = NO_DIRECTION_ON_ROBOT; 
              rd < adjacent_vertices.size(); ++rd) {
            for (int nrl = NO_ROBOT; nrl < robot_vertices.size(); ++nrl) {
              state_cache_[count].graph_id = graph_id;
              state_cache_[count].direction = direction;
              state_cache_[count].num_robots_left = robots;
              if (rd < 0) {
                state_cache_[count].current_robot_direction = rd;
              } else {
                state_cache_[count].current_robot_direction = 
                  adjacent_vertices[rd];
              }
              if (nrl < 0) {
                state_cache_[count].next_robot_location = nrl;
              } else {
                state_cache_[count].next_robot_location = robot_vertices[nrl];
              }
            }
          }
        }
      }
    }
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
    actions.push_back(Action(DO_NOTHING,0));

    // If state does not have a future robot, allow placing a robot in a future
    // location
    if (state.next_robot_location == NO_ROBOT) {
      BOOST_FOREACH(int id, robot_vertices_map_[state.graph_id]) {
        actions.push_back(Action(PLACE_FUTURE_ROBOT, id); 
      }
    }

    // If state has a robot at its current location, but the robot is not 
    // pointing to a graph id, allow placing a direction on that robot
    if (state.current_robot_direction == NO_DIRECTION_ON_ROBOT) {
      BOOST_FOREACH(int id, adjacent_vertices_map_[state.graph_id]) {
        actions.push_back(Action(PLACE_ROBOT, id));
      }
    }
  }

  std::vector<Action>& PersonModel2::getActionsAtState(
      const State2& state) {
    return action_cache_[state];
  }

  void PersonModel2::initializeNextStateCache() {

    next_state_cache_.clear();
    for (int i = 0; i < num_vertices_; ++i) {
      for (int j = 0; j < num_directions_; ++j) {
        BOOST_FOREACH(int id, adjacent_vertices_map_[i]) {
          int direction = computeNextDirection(j, i, id);
          next_state_cache_[i * num_directions_ + j].push_back(
              std::make_pair(id, direction));
        }
      }
    }
    
    ns_distribution_cache_.clear();
    BOOST_FOREACH(const State& state, state_cache_) {
      std::vector<Action>& actions = getActionsAtState(state);
      BOOST_FOREACH(const Action& action, actions) {
        constructTransitionProbabilities(state, action, 
            ns_distribution_cache_[state][action]);
      }
    }

  }

  void getNextStates(const State2& state, const Action& action, 
      std::vector<State2>& states) {

    states.clear();

    if (isTerminalState(state)) {
      return; // no next states
    }

    if (action.type == PLACE_ROBOT) {
      State2 next_state = state;
      next_state.current_robot_direction = action.graph_id;
      states.push_back(next_state);
      return;
    }
    if (action.type == PLACE_FUTURE_ROBOT) {
      State2 next_state = state;
      next_state.next_robot_location = action.graph_id;
      next_state.num_robots_left--;
      states.push_back(next_state);
      return;
    }
   
    // If the person is actually planning on moving, this one is going to be fun 
    // Get all adjacent ids and the directions if we move to those ids
    std::vector<std::pair<int, int> >& next_states = 
      next_state_cache_[state.graph_id * num_directions_ + state.directions];
    typedef std::pair<int, int> int2pair;
    BOOST_FOREACH(int2pair& next_loc, next_states) {
      State2 next_state;
      next_state.graph_id = next_loc.first();
      next_state.direction = next_loc.second();
      next_state.num_robots_left = state.num_robots_left;

      // See if the location we moved to is where the next robot is
      if (next_state.graph_id == state.next_robot_location) {
        // Transition to the state where the current location has a robot, but
        // no direction
        next_state.current_robot_direction = NO_DIRECTION_ON_ROBOT;
        next_state.next_robot_location = NO_ROBOT; // This robot no longer needs to be tracked
      } else {
        // See if the robot is still visible from the current location - you do not want to move
        // to a state where the robot is not visible
        if (std::find(robot_vertices_map_[next_state.graph_id].begin(),
              robot_vertices_map_[next_state.graph_id].end(), state.next_robot_location) 
            == robot_vertices_map_[next_state.graph_id].end()) { 
          // The person moved in an unexpected manner, the robot gets decommisioned
          next_state.next_robot_location == NO_ROBOT;
        } else {
          next_state.next_robot_location == state.next_robot_location;
        }
        next_state.current_robot_direction = NO_ROBOT;
      }
      states.push_back(next_state);
    }
  }

  void PersonModel2::constructTransitionProbabilities(State2 state, 
      Action action, std::vector<float>& probabilities) {

    probabilities.clear();

    if (isTerminalState(state)) {
      return;
    }

    if (action.type == PLACE_ROBOT || action.type == PLACE_FUTURE_ROBOT) {
      // This is a deterministic single transition
      probabilities.push_back(1.0);
      return;
    }

    // If the person is going to be making the transition here, figure out
    // next state probabilities using the person model (i.e action = DO_NOTHING)
    
    // In this simple MDP formulation, the action should induce a desired 
    // direction for the person to be walking to.
    // TODO: Take into account the location of next_robot_location
    float expected_direction, sigma_sq;
    if (state.current_robot_direction != NO_ROBOT || 
        state.current_robot_direction != NO_DIRECTION_ON_ROBOT // shouldn't really happen - VI messed up
        ) {
      expected_direction = 
        getAngleFromStates(state.graph_id, action.graph_id);
      sigma_sq = 0.05;
    } else {
      expected_direction = 
        getAngleFromDirection(state.direction);
      sigma_sq = 0.05;
    }

    // Now compute the weight of each next state. Get the favored direction
    // and compute transition probabilities
    std::vector<State2> next_states;
    getNextStates(state, action, next_states);
    float probability_sum = 0;
    size_t last_non_zero_probability = 0;
    BOOST_FOREACH(const State2& next_state, next_states) {

      float next_state_direction = 
        getAngleFromStates(state.graph_id, next_state.graph_id);

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
    }

    // Normalize probabilities. Ensure sum == 1 with last non zero probability 
    float normalized_probability_sum = 0;
    for (size_t probability_counter = 0; 
        probability_counter < probabilities.size();
        ++probability_counter) {
      probabilities[probability_counter] /= probability_sum;
      normalized_probability_sum += probabilities[probability_counter];
    }
    probabilities[last_non_zero_probability] += 1 - normalized_probability_sum;

  }

  std::vector<float>& PersonModel2::getTransitionProbabilities(
      const State2& state, const Action& action) {
    return ns_distribution_cache_[state][action];
  }

  size_t PersonModel2::computeNextDirection(size_t dir, size_t graph_id, 
      size_t next_graph_id) {
    float angle = getAngleFromStates(graph_id, next_graph_id);
    return getDirectionFromAngle(angle);
  }

  float PersonModel2::getAngleFromStates(size_t graph_id, size_t next_graph_id) {

    topological_mapper::Graph::vertex_descriptor v = 
      boost::vertex(graph_id, graph_);
    topological_mapper::Graph::vertex_descriptor next_v =
      boost::vertex(next_graph_id, graph_);

    return atan2f(graph_[next_v].location.y - graph_[v].location.y,
        graph_[next_v].location.x - graph_[v].location.x);
  }

  float PersonModel2::getDistanceFromStates(size_t graph_id, size_t next_graph_id) {
    topological_mapper::Graph::vertex_descriptor v = 
      boost::vertex(graph_id, graph_);
    topological_mapper::Graph::vertex_descriptor next_v =
      boost::vertex(next_graph_id, graph_);

    return topological_mapper::getMagnitude(
        graph_[next_v].location - graph_[v].location);
  }

  size_t PersonModel2::getDirectionFromAngle(float angle) {
    angle = angle + M_PI / num_directions_;
    while (angle < 0) angle += 2 * M_PI;
    while (angle >= 2 * M_PI) angle -= 2 * M_PI;
    return (angle * num_directions_) / (2 * M_PI);
  }

  float PersonModel2::getAngleFromDirection(size_t dir) {
    return ((2 * M_PI) / num_directions_) * dir;
  }

} /* bwi_exp1 */
