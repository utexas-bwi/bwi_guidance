#include <bwi_exp1_solver/mdp.h>
#include <topological_mapper/map_loader.h>

using namespace bwi_exp1;

void testValueIteration(topological_mapper::Graph& graph) {

  size_t goal_idx = 33;
  size_t num_vertices = boost::num_vertices(graph);
  State s;

  std::vector<bwi_exp1::State> state_space;
  bwi_exp1::populateStateSpace(graph, state_space);

  std::cout << "Size of state cache: " << sizeof(State) * state_space.size() << std::endl; 

  // for each state, get the cached actions (independent of current direction)
  std::vector<std::vector<Action> > action_space;
  action_space.resize(num_vertices * 2);
  long size_action_space = 0;
  for (size_t graph_id = 0; graph_id < num_vertices; ++graph_id) {
    s.graph_id = graph_id;
    for (size_t robots_remaining = 0; robots_remaining <= 1; ++robots_remaining) {
      s.num_robots_left = robots_remaining;
      std::vector<Action>& actions =
        action_space[graph_id * 2 + robots_remaining];
      getActionsAtState(s, graph, actions);
      size_action_space += actions.size() * sizeof(Action);
    }
  }

  std::cout << "Size of action cache: " << size_action_space << std::endl; 
  std::cout << "Testing action cache..." << std::endl; 

  // Test action cache 
  size_t action_cache_test_idx = 76;
  for (size_t robots_remaining = 0; robots_remaining <= 1; ++robots_remaining) {
    std::cout << " Actions at " << action_cache_test_idx << " with robots_remaining: " << robots_remaining << std::endl;
    std::cout << " rr != 0: " << (robots_remaining != 0) << std::endl;
    std::vector<Action>& actions = action_space[action_cache_test_idx * 2 + (robots_remaining != 0)];
    for (size_t action_idx = 0; action_idx < actions.size(); ++action_idx) {
      Action action = actions[action_idx];
      std::cout << "  - (" << action.type << ", " << action.graph_id << ")" << std::endl;
    }
  }

  // for each state, get the cached next states
  std::vector<std::vector<size_t> > next_state_space;
  next_state_space.resize(num_vertices * NUM_DIRECTIONS * (MAX_ROBOTS + 1));
  long size_next_state_space = 0;
  for (size_t state_id = 0; state_id < state_space.size(); ++state_id) {
    getNextStatesAtState(state_space[state_id], graph, next_state_space[state_id]);
    size_next_state_space += sizeof(size_t) * next_state_space[state_id].size();
  }

  std::cout << "Size of next state cache: " << size_next_state_space << 
    std::endl;

  std::vector<std::vector<std::vector<float> > > probability_space;
  probability_space.resize(num_vertices * NUM_DIRECTIONS * (MAX_ROBOTS + 1));
  long size_transition_probability_space = 0;
  for (size_t state_id = 0; state_id < state_space.size(); ++state_id) {
    State& state = state_space[state_id];
    std::vector<Action>& actions = action_space[state.graph_id * 2 + (state.num_robots_left != 0)];

    // if (state.graph_id == 0 && state.direction == 0 && state.num_robots_left == 0) {
    //   std::cout << "At State (" << state.graph_id << ", " << state.direction << ", " << state.num_robots_left << ")" << std::endl;
    //   for (size_t action_counter = 0; action_counter < actions.size(); ++action_counter) {
    //     Action& action = actions[action_counter];
    //     std::cout << "  - An action is (" << action.type << ", " << action.graph_id << ")" << std::endl;
    //   }
    // }

    probability_space[state_id].resize(action_space[state.graph_id * 2 + (state.num_robots_left != 0)].size());
    for (size_t action_id = 0; action_id < actions.size(); ++action_id) {
      std::vector<size_t> next_states;
      getTransitionProbabilities(state, state_space, graph, actions[action_id], next_states, probability_space[state_id][action_id]);
      size_transition_probability_space += sizeof(float) * next_states.size();
    }
  }

  std::cout << "Size of next probability space cache: " << size_transition_probability_space << 
    std::endl;

  // Now perform value iteration
  std::vector<float> value_space(state_space.size(), 0);
  size_t count = 0;
  bool change = true;
  while(change) {
    change = false;
    count++;
    std::cout << "Iteration " << count << std::endl;
    for (size_t state_id = 0; state_id < value_space.size(); ++state_id) {
      State& state = state_space[state_id];
      if (isTerminalState(state, goal_idx))
        continue;
      std::vector<size_t>& next_states = next_state_space[state_id];
      std::vector<Action>& actions = action_space[state.graph_id * 2 + (state.num_robots_left != 0)];
      float value = -std::numeric_limits<float>::max();
      for (size_t action_id = 0; action_id < actions.size(); ++action_id) {
        float action_value = 0;
        std::vector<float>& probability = probability_space[state_id][action_id];

        for (size_t next_state_counter = 0; next_state_counter < next_states.size(); ++next_state_counter) {
          action_value += probability[next_state_counter] * (getReward(state_space, state_id, next_states[next_state_counter], graph) + GAMMA * value_space[next_states[next_state_counter]]);

        }
        if (action_value > value) {
          value = action_value;
        }
      }
      if (count == 1) {
        std::cout << value << std::endl;
      }
      change = change || (value_space[state_id] != value);
      value_space[state_id] = value;
    }
  }

  // Now perform a max walk from start state to goal state
  size_t start_idx = 76;
  for (size_t starting_robots = MAX_ROBOTS; starting_robots >= 1; --starting_robots) {
    std::cout << std::endl << "SETUP with STARTING ROBOTS = " << starting_robots << std::endl;
    size_t current_state_idx = constructStateIndex(start_idx, 12, starting_robots);
    while (state_space[current_state_idx].graph_id != goal_idx) {
      State& current_state = state_space[current_state_idx];
      std::cout << "At State (" << current_state.graph_id << ", " << current_state.direction << ", " << current_state.num_robots_left << ")" << std::endl;
      std::cout << "Value of this state: " << value_space[current_state_idx] << std::endl;

      std::cout << " Actions available: " << std::endl;
      std::vector<Action>& actions = action_space[current_state.graph_id * 2 + (current_state.num_robots_left != 0)];
      for (size_t action_idx = 0; action_idx < actions.size(); ++action_idx) {
        Action action = actions[action_idx];
        std::cout << "  - #" << action_idx << " (" << action.type << ", " << action.graph_id << ")" << std::endl;
      }
      std::cout << "Choice: ";
      int choice;
      std::cin >> choice;
      Action& action = actions[choice];
      std::vector<size_t> next_states;
      std::vector<float> probabilities;
      getTransitionProbabilities(current_state, state_space, graph, action, next_states, probabilities);
      size_t most_likely_transition = (size_t) -1;
      float most_likely_transition_prob = -0.1;

      for (size_t next_state_counter = 0; next_state_counter < next_states.size(); ++next_state_counter) {
        size_t next_state_idx = next_states[next_state_counter];
        State& next_state = state_space[next_state_idx];
        std::cout << "  - Leads to State (" << next_state.graph_id << ", " << next_state.direction << ", " << next_state.num_robots_left << ") with probability " << probabilities[next_state_counter] << std::endl;

        if (probabilities[next_state_counter] > most_likely_transition_prob) {
          most_likely_transition_prob = probabilities[next_state_counter];
          most_likely_transition = next_state_idx;
        }
      }
      current_state_idx = most_likely_transition;
      std::cout << " - Making transition to most likely state for this best action" << std::endl; 

    }
    std::cout << "VALUE of final state: " << value_space[current_state_idx] << std::endl;
  
  }
}

void testActionChoices(topological_mapper::Graph& graph) {

  State state;
  state.graph_id = 16;
  state.num_robots_left = 1;
  std::vector<Action> actions;
  getActionsAtState(state, graph, actions);

  std::cout << "At State (" << state.graph_id << ", " << state.direction << ", " << state.num_robots_left << ")" << std::endl;
  for (size_t action_counter = 0; action_counter < actions.size(); ++action_counter) {
    Action& action = actions[action_counter];
    std::cout << "  - An action is (" << action.type << ", " << action.graph_id << ")" << std::endl;
  }

}

void testNextDirectionComputation(topological_mapper::Graph& graph) {

  size_t direction = 12;
  size_t current_state = 78;
  size_t next_state = 79;
  size_t next_direction;

  next_direction = bwi_exp1::computeNextDirection(direction, current_state, next_state, graph);
  std::cout << "(" << current_state << ", " << direction << ") -> " <<
      "(" << next_state << ", " << next_direction << ")" << std::endl;

  current_state = next_state;
  direction = next_direction;
  next_state = 65;
  next_direction = bwi_exp1::computeNextDirection(direction, current_state, next_state, graph);
  std::cout << "(" << current_state << ", " << direction << ") -> " <<
      "(" << next_state << ", " << next_direction << ")" << std::endl;

  current_state = next_state;
  direction = next_direction;
  next_state = 57;
  next_direction = bwi_exp1::computeNextDirection(direction, current_state, next_state, graph);
  std::cout << "(" << current_state << ", " << direction << ") -> " <<
      "(" << next_state << ", " << next_direction << ")" << std::endl;

  current_state = next_state;
  direction = next_direction;
  next_state = 55;
  next_direction = bwi_exp1::computeNextDirection(direction, current_state, next_state, graph);
  std::cout << "(" << current_state << ", " << direction << ") -> " <<
      "(" << next_state << ", " << next_direction << ")" << std::endl;

  current_state = next_state;
  direction = next_direction;
  next_state = 51;
  next_direction = bwi_exp1::computeNextDirection(direction, current_state, next_state, graph);
  std::cout << "(" << current_state << ", " << direction << ") -> " <<
      "(" << next_state << ", " << next_direction << ")" << std::endl;

  current_state = next_state;
  direction = next_direction;
  next_state = 33;
  next_direction = bwi_exp1::computeNextDirection(direction, current_state, next_state, graph);
  std::cout << "(" << current_state << ", " << direction << ") -> " <<
      "(" << next_state << ", " << next_direction << ")" << std::endl;

}

int main(int argc, char** argv) {

  if (argc < 3) {
    std::cerr << "USAGE: " << argv[0] 
        << " <yaml-map-file> <yaml-graph-file>" << std::endl;
    return -1;
  }

  topological_mapper::MapLoader mapper(argv[1]);
  topological_mapper::Graph graph;
  nav_msgs::MapMetaData info;
  mapper.getMapInfo(info);
  topological_mapper::readGraphFromFile(argv[2], info, graph);

  //testNextDirectionComputation(graph);
  //testActionChoices(graph);
  testValueIteration(graph);

  return 0;
}
