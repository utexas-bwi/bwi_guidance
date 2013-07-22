#include <bwi_exp1_solver/simple_model.h>

namespace bwi_exp1 {

  Action::Action() : type(DO_NOTHING), graph_id(0) {}
  Action::Action(ActionType a, size_t g) : type(a), graph_id(g) {}

  PersonModel::PersonModel(const topological_mapper::Graph& graph, size_t goal_idx) : graph_(graph), goal_idx_(goal_idx) {
    initializeStateSpace();
    initializeActionCache();
    initializeNextStateCache();
  }

  /** Update the MDP model with a vector of experiences. */
  virtual bool updateWithExperiences(std::vector<experience> &instances) = 0;

  /** Update the MDP model with a single experience. */
  virtual bool updateWithExperience(experience &instance) = 0;

  /** Get the predictions of the MDP model for a given state action */
  virtual float getStateActionInfo(const std::vector<float> &state, int action, StateActionInfo* retval) = 0;

  /** Get a copy of the MDP Model */
  virtual PersonModel* getCopy() = 0;
  virtual ~PersonModel() {};

  state_t PersonModel::canonicalizeState(uint32_t graph_id, uint32_t direction, uint32_t robots_remaining) {
    return graph_id * num_directions_ * (max_robots_ + 1) + 
      direction * (max_robots_ + 1) +
      robots_remaining;
  }

  void PersonModel::initializeStateSpace() {
    // TODO: Initialize these using command line arguments
    num_vertices_ = boost::num_vertices(graph_);
    num_directions_ = 16;
    max_robots_ = 5;

    state_cache_.resize(getStateSpaceSize(num_vertices));
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
    for (size_t graph_id = 0; graph_id < num_vertices; ++graph_id) {
      for (size_t robots_remaining = 0; robots_remaining <= 1; ++robots_remaining) {
        state_t state = canonicalizeState(graph_id, 0, robots_remaining);
        std::vector<Action>& actions =
          action_space[graph_id * 2 + robots_remaining];
        constructActionsAtState(state, actions);
      }
    }
  }

  void PersonModel::constructActionsAtState(state_t state, std::vector<Action>& actions) {
    actions.clear();
    actions.push_back(Action(DO_NOTHING,0));

    boost::property_map<Graph, boost::vertex_index_t>::type 
      indexmap = boost::get(boost::vertex_index, graph);

    State& s = state_cache_[state];
    // Add place robot actions to all adjacent vertices if robots remaining
    if (s.num_robots_left != 0) {
      Graph::vertex_descriptor v = boost::vertex(s.graph_id, graph);
      Graph::adjacency_iterator ai, aend;
      for (boost::tie(ai, aend) = boost::adjacent_vertices(v, graph); 
          ai != aend; ++ai) {
        actions.push_back(Action(PLACE_ROBOT, indexmap[*ai]));
      }
    }
  }

  std::vector<Action>& getActionsAtState(state_t state) {
    State& s = state_cache_[state];
    return action_cache_[s.graph_id * 2 + (s.num_robots_left != 0)];
  }

  void PersonModel::initializeNextStateCache() {
    next_state_cache_.resize(getStateSpaceSize());
    for (state_t state = 0; state < getStateSpaceSize(); ++state) {
      std::vector<state_t>& next_states = next_state_cache_[state];
      constructNextStatesAtState(state, next_state);
    }
    ns_distribution_cache_.resize(getStateSpaceSize());
    for (state_t state = 0; state < getStateSpaceSize(); ++state) {
      std::vector<Action>& actions = getActionsAtState(state);
      

    }
  }

  void PersonModel::constructNextStatesAtState(state_t state, 
      std::vector<state_t>& next_states) {
    State& s = state_cache_[state];
    next_states.clear();
    boost::property_map<Graph, boost::vertex_index_t>::type 
        indexmap = boost::get(boost::vertex_index, graph);

    // Get all the adjacent vertices
    Graph::vertex_descriptor v = boost::vertex(state.graph_id, graph);
    std::vector<size_t> adjacent_idxs;
    Graph::adjacency_iterator ai, aend;
    for (boost::tie(ai, aend) = boost::adjacent_vertices(v, graph); 
        ai != aend; ++ai) {
      adjacent_idxs.push_back(indexmap[*ai]);
    }

    // Now compute all the possible next states
    for (size_t a = 0; a < adjacent_idxs.size(); ++a) {
      uint32_t next_dir = computeNextDirection(state.direction, state.graph_id,
          adjacent_idxs[a], graph);

      // Add next state if we do nothing
      next_states.push_back(
          constructStateIndex(adjacent_idxs[a], next_dir, state.num_robots_left));

      // Add next state if we place a robot
      if (state.num_robots_left != 0) {
        next_states.push_back(
            constructStateIndex(adjacent_idxs[a], next_dir, state.num_robots_left - 1));
      }
    }
  }

  void PersonModel::

  size_t PersonModel::getStateSpaceSize() {
    return num_vertices_ * num_directions_ * (max_robots_ + 1);
  }

  void PersonModel::getTransitionProbabilities(state_t state_id, action_t action_id, 
      std::vector<state_t>& next_states, std::vector<float>& probabilities) {

    // Get all possible next state
    getNextStatesAtState(state_id, next_states);

    State& state = state_cache_[state_id];
    Action& action = action_cache_[action_id];

    // In this simple MDP formulation, the action should induce a desired 
    // direction for the person to be walking to.
    float expected_direction, sigma_sq;
    if (action.type == PLACE_ROBOT) {
      expected_direction = 
        getAngleFromStates(graph, state.graph_id, action.graph_id);
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
        getAngleFromStates(graph, state.graph_id, next_state.graph_id);

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

} /* bwi_exp1 */
