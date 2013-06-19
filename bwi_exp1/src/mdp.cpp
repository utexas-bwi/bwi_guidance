#include <bwi_exp1/mdp.h>

namespace bwi_exp1 {
 
  size_t computeNextDirection(size_t dir, size_t graph_id, 
      size_t next_graph_id, const Graph &graph) {

    float x, y;
    getXYDirectionFromStates(graph, graph_id, next_graph_id, x, y);
    //std::cout << x << " " << y << std::endl;
    
    float max_value = ((float)GRID_SIZE - 1.0) / 2.0;
    float offset = ((GRID_SIZE + 1) % 2) * 0.5;

    float x_curr = (dir % GRID_SIZE) - max_value;
    float y_curr = (dir / GRID_SIZE) - max_value;

    //std::cout << dir << " -> " << x_curr << " " << y_curr << std::endl;

    float x_net = x_curr / 2.0 + x;
    x_net = std::min(max_value, x_net);
    x_net = std::max(-max_value, x_net);
    float y_net = y_curr / 2.0 + y;
    y_net = std::min(max_value, y_net);
    y_net = std::max(-max_value, y_net);

    int x_idx = round(x_net + offset) - round (-max_value + offset);
    int y_idx = round(y_net + offset) - round (-max_value + offset);

    return y_idx * GRID_SIZE + x_idx;
  }

  void getActionsAtState(const State& state, const Graph& graph, 
      std::vector<Action>& actions) {

    actions.clear();
    actions.push_back(Action(DO_NOTHING,0));

    boost::property_map<Graph, boost::vertex_index_t>::type 
        indexmap = boost::get(boost::vertex_index, graph);

    // Add place robot actions to all adjacent vertices if robots remaining
    if (state.num_robots_left != 0) {
      Graph::vertex_descriptor v = boost::vertex(state.graph_id, graph);
      Graph::adjacency_iterator ai, aend;
      for (boost::tie(ai, aend) = boost::adjacent_vertices(v, graph); 
          ai != aend; ++ai) {
        actions.push_back(Action(PLACE_ROBOT, indexmap[*ai]));
      }
    }

  }

  void getNextStatesAtState(const State& state, const Graph& graph, 
      std::vector<size_t>& next_states) {

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
      size_t next_dir = computeNextDirection(state.direction, state.graph_id,
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

  void populateStateSpace(const Graph &graph, 
      std::vector<State>& state_space) {

    size_t num_vertices = boost::num_vertices(graph);
    state_space.resize(getStateSpaceSize(num_vertices));

    for (size_t i = 0; i < num_vertices; ++i) {
      // Add the relevant states
      for (size_t dir = 0; dir < NUM_DIRECTIONS; ++dir) {
        for (size_t robots_remaining = 0; robots_remaining <= MAX_ROBOTS; 
            ++robots_remaining) {

          size_t state_idx = constructStateIndex(i, dir, robots_remaining);
          state_space[state_idx].graph_id = i;
          state_space[state_idx].direction = dir;
          state_space[state_idx].num_robots_left = robots_remaining;
          
        }
      }
    }
  }

  void getTransitionProbabilities(const State& state, 
      const std::vector<State>& state_space, 
      const Graph& graph, const Action& action, 
      std::vector<size_t>& next_states, std::vector<float>& probabilities) {

    // Get all possible next state
    getNextStatesAtState(state, graph, next_states);

    // In this simple MDP formulation, the action should induce a desired 
    // direction for the person to be walking to.
    float expected_direction, sigma_sq;
    if (action.type == PLACE_ROBOT) {
      expected_direction = 
        getAngleFromStates(graph, state.graph_id, action.graph_id);
      sigma_sq = 0.2;
    } else {
      expected_direction = 
        getAngleFromDirection(state.direction);
      sigma_sq = 1.0;
    }

    // Now compute the weight of each next state. Get the favored direction
    // and compute transition probabilities
    probabilities.clear();
    float probability_sum = 0;
    size_t last_non_zero_probability = 0;
    for (size_t next_state_counter = 0; next_state_counter < next_states.size();
        ++next_state_counter) {

      const State& next_state = state_space[next_states[next_state_counter]];
      
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

    // Normalize probabilities. Add an epsilon to last non zero probability to 
    // ensure sum > 1 to avoid floating point errors
    for (size_t probability_counter = 0; 
        probability_counter < probabilities.size();
        ++probability_counter) {
      probabilities[probability_counter] /= probability_sum;
    }
    probabilities[last_non_zero_probability] += 0.1; // add epsilon

  }

}
