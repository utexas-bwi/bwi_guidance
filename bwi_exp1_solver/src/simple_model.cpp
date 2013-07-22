#include <bwi_exp1_solver/simple_model.h>
#include <cmath>

namespace bwi_exp1 {

  Action::Action() : type(DO_NOTHING), graph_id(0) {}
  Action::Action(ActionType a, size_t g) : type(a), graph_id(g) {}

  PersonModel::PersonModel(const topological_mapper::Graph& graph, size_t goal_idx) : graph_(graph), goal_idx_(goal_idx) {
    initializeStateSpace();
    initializeActionCache();
    initializeNextStateCache();
  }

  /** Update the MDP model with a vector of experiences. */
  bool PersonModel::updateWithExperiences(std::vector<experience> &instances) {
    throw std::runtime_error("Updating person model is not supported!!");
  }

  /** Update the MDP model with a single experience. */
  bool PersonModel::updateWithExperience(experience &instance) {
    throw std::runtime_error("Updating person model is not supported!!");
  }

  PersonModel* PersonModel::getCopy() {
    throw std::runtime_error("Copy person model is not supported!!");
  }

  /** Get the predictions of the MDP model for a given state action */
  float PersonModel::getStateActionInfo(const std::vector<float> &state, int action, StateActionInfo* retval) {
    retval->transitionProbs.clear();
    retval->known = false;
    state_t s = canonicalizeState(state);
    State &s_deref = state_cache_[s]; 
    action_t a = canonicalizeAction(action);
    if (state_cache_[s].graph_id == goal_idx_) {
      retval->reward = 0;
      retval->termProb = 1.0;
    } else {
      std::vector<Action> actions = getActionsAtState(s);
      if (a >= actions.size()) {
        // this action is not well defined for this state, should return a negative confidence
        return -1.0f;
      }
      Action& a_deref = actions[a];
      std::vector<state_t> next_states = getNextStatesAtState(s);
      std::vector<float> transition_probabilities = getTransitionProbabilities(s, a);
      retval->reward = 0;
      for (size_t ns = 0; ns < next_states.size(); ++ns) {
        state_t &next_state = next_states[ns]; 
        State &ns_deref = state_cache_[ns]; 
        std::vector<float> ns_continuous;
        produceContinuousState(next_state, ns_continuous);
        retval->transitionProbs[ns_continuous] = transition_probabilities[ns];
        retval->reward += transition_probabilities[ns] *
            -getDistanceFromStates(s_deref.graph_id, ns_deref.graph_id);
      }
      if (a_deref.type == PLACE_ROBOT) {
        retval->reward += -500;
      }
      retval->termProb = 0;
    }
    retval->known = true;
  }

  state_t PersonModel::canonicalizeState(uint32_t graph_id, uint32_t direction, uint32_t robots_remaining) const {
    return graph_id * num_directions_ * (max_robots_ + 1) + 
      direction * (max_robots_ + 1) +
      robots_remaining;
  }

  void PersonModel::produceContinuousState(state_t state_id,
      std::vector<float>& state) {
    State& s = state_cache_[state_id];
    state.resize(3);
    state[0] = (float) s.graph_id;
    state[1] = (float) s.direction;
    state[1] = (float) s.num_robots_left;
  }

  state_t PersonModel::canonicalizeState(const std::vector<float> &state) const {
    return canonicalizeState(lrint(state[0]), lrint(state[1]), lrint(state[2]));
  }

  action_t PersonModel::canonicalizeAction(int action) const {
    return action;
  }

  void PersonModel::initializeStateSpace() {
    // TODO: Initialize these using command line arguments
    num_vertices_ = boost::num_vertices(graph_);
    num_directions_ = 16;
    max_robots_ = 5;

    state_cache_.resize(getStateSpaceSize());
    for (uint32_t i = 0; i < num_vertices_; ++i) {
      for (uint32_t dir = 0; dir < num_directions_; ++dir) {
        for (uint32_t robots_remaining = 0; robots_remaining <= max_robots_; 
            ++robots_remaining) {
          state_t state = canonicalizeState(i, dir, robots_remaining);
          state_cache_[state].graph_id = i;
          state_cache_[state].direction = dir;
          state_cache_[state].num_robots_left = robots_remaining;
        }
      }
    }
  }

  void PersonModel::initializeActionCache() {
    action_cache_.resize(num_vertices_ * 2);
    for (size_t graph_id = 0; graph_id < num_vertices_; ++graph_id) {
      for (size_t robots_remaining = 0; robots_remaining <= 1; ++robots_remaining) {
        state_t state = canonicalizeState(graph_id, 0, robots_remaining);
        std::vector<Action>& actions =
          action_cache_[graph_id * 2 + robots_remaining];
        constructActionsAtState(state, actions);
      }
    }
  }

  void PersonModel::constructActionsAtState(state_t state, std::vector<Action>& actions) {
    actions.clear();
    actions.push_back(Action(DO_NOTHING,0));

    boost::property_map<
      topological_mapper::Graph, boost::vertex_index_t>::type 
      indexmap = boost::get(boost::vertex_index, graph_);

    State& s = state_cache_[state];
    // Add place robot actions to all adjacent vertices if robots remaining
    if (s.num_robots_left != 0) {
      topological_mapper::Graph::vertex_descriptor v = 
        boost::vertex(s.graph_id, graph_);
      topological_mapper::Graph::adjacency_iterator ai, aend;
      for (boost::tie(ai, aend) = boost::adjacent_vertices(v, graph_); 
          ai != aend; ++ai) {
        actions.push_back(Action(PLACE_ROBOT, indexmap[*ai]));
      }
    }
  }

  std::vector<Action>& PersonModel::getActionsAtState(state_t state) {
    State& s = state_cache_[state];
    return action_cache_[s.graph_id * 2 + (s.num_robots_left != 0)];
  }

  void PersonModel::initializeNextStateCache() {
    next_state_cache_.resize(getStateSpaceSize());
    for (state_t state = 0; state < getStateSpaceSize(); ++state) {
      std::vector<state_t>& next_states = next_state_cache_[state];
      constructNextStatesAtState(state, next_states);
    }
    ns_distribution_cache_.resize(getStateSpaceSize());
    for (state_t state = 0; state < getStateSpaceSize(); ++state) {
      std::vector<Action>& actions = getActionsAtState(state);
      ns_distribution_cache_[state].resize(actions.size());
      for (size_t action_id = 0; action_id < actions.size(); ++action_id) {
        constructTransitionProbabilities(state, action_id, 
            ns_distribution_cache_[state][action_id]);
      }
    }
  }

  void PersonModel::constructNextStatesAtState(state_t state_id, 
      std::vector<state_t>& next_states) {
    State& state = state_cache_[state_id];
    next_states.clear();
    boost::property_map<topological_mapper::Graph, boost::vertex_index_t>::type 
        indexmap = boost::get(boost::vertex_index, graph_);

    // Get all the adjacent vertices
    topological_mapper::Graph::vertex_descriptor v = 
      boost::vertex(state.graph_id, graph_);
    std::vector<size_t> adjacent_idxs;
    topological_mapper::Graph::adjacency_iterator ai, aend;
    for (boost::tie(ai, aend) = boost::adjacent_vertices(v, graph_); 
        ai != aend; ++ai) {
      adjacent_idxs.push_back(indexmap[*ai]);
    }

    // Now compute all the possible next states
    for (size_t a = 0; a < adjacent_idxs.size(); ++a) {
      uint32_t next_dir = computeNextDirection(state.direction, state.graph_id,
          adjacent_idxs[a]);

      // Add next state if we do nothing
      next_states.push_back(
          canonicalizeState(adjacent_idxs[a], next_dir, state.num_robots_left));

      // Add next state if we place a robot
      if (state.num_robots_left != 0) {
        next_states.push_back(
            canonicalizeState(adjacent_idxs[a], next_dir, state.num_robots_left - 1));
      }
    }
  }

  std::vector<state_t>& PersonModel::getNextStatesAtState(state_t state) {
    return next_state_cache_[state];
  }

  size_t PersonModel::getStateSpaceSize() const {
    return num_vertices_ * num_directions_ * (max_robots_ + 1);
  }

  void PersonModel::constructTransitionProbabilities(state_t state_id, 
      action_t action_id, std::vector<float>& probabilities) {

    // Get all possible next state
    std::vector<state_t>& next_states = getNextStatesAtState(state_id);

    State& state = state_cache_[state_id];
    std::vector<Action>& actions = getActionsAtState(state_id);
    Action& action = actions[action_id];

    // In this simple MDP formulation, the action should induce a desired 
    // direction for the person to be walking to.
    float expected_direction, sigma_sq;
    if (action.type == PLACE_ROBOT) {
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
    probabilities.clear();
    float probability_sum = 0;
    size_t last_non_zero_probability = 0;
    for (size_t next_state_counter = 0; next_state_counter < next_states.size();
        ++next_state_counter) {

      const State& next_state = state_cache_[next_states[next_state_counter]];

      // Some next states cannot be produced by certain actions
      if ((action.type == DO_NOTHING && 
            next_state.num_robots_left == state.num_robots_left - 1) ||
          (action.type == PLACE_ROBOT &&
           next_state.num_robots_left == state.num_robots_left)) {

        probabilities.push_back(0);
        continue;
      }

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

    if (state.graph_id == 0 && state.direction == 0 && state.num_robots_left == 0) {
      std::cout << "Sum is " << normalized_probability_sum << " across " << next_states.size() << " states." << std::endl;
      std::cout << probabilities[0] << " " << probabilities[1] << std::endl;
    }

  }

  std::vector<float>& PersonModel::getTransitionProbabilities(
      state_t state_id, action_t action_id) {
    return ns_distribution_cache_[state_id][action_id];
  }

  size_t PersonModel::computeNextDirection(size_t dir, size_t graph_id, 
      size_t next_graph_id) {
    float angle = getAngleFromStates(graph_id, next_graph_id);
    return getDirectionFromAngle(angle);
  }

  float PersonModel::getAngleFromStates(size_t graph_id, size_t next_graph_id) {

    topological_mapper::Graph::vertex_descriptor v = 
      boost::vertex(graph_id, graph_);
    topological_mapper::Graph::vertex_descriptor next_v =
      boost::vertex(next_graph_id, graph_);

    return atan2f(graph_[next_v].location.y - graph_[v].location.y,
        graph_[next_v].location.x - graph_[v].location.x);
  }

  float PersonModel::getDistanceFromStates(size_t graph_id, size_t next_graph_id) {
    topological_mapper::Graph::vertex_descriptor v = 
      boost::vertex(graph_id, graph_);
    topological_mapper::Graph::vertex_descriptor next_v =
      boost::vertex(next_graph_id, graph_);

    return cv::norm(graph_[next_v].location - graph_[v].location);
  }

  size_t PersonModel::getDirectionFromAngle(float angle) {
    angle = angle + M_PI / num_directions_;
    while (angle < 0) angle += 2 * M_PI;
    while (angle >= 2 * M_PI) angle -= 2 * M_PI;
    return (angle * num_directions_) / (2 * M_PI);
  }

  float PersonModel::getAngleFromDirection(size_t dir) {
    return ((2 * M_PI) / num_directions_) * dir;
  }

} /* bwi_exp1 */
