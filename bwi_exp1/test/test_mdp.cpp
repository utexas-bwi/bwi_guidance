#include <bwi_exp1/mdp.h>
#include <topological_mapper/map_loader.h>

using namespace bwi_exp1;

void testStateSpacePopulation(topological_mapper::Graph& graph) {

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

  // for each state, get the cached next states
  std::vector<std::vector<size_t> > next_state_space;
  next_state_space.resize(num_vertices * NUM_DIRECTIONS * (MAX_ROBOTS + 1));
  long size_next_state_space = 0;
  for (size_t state_id = 0; state_id < state_space.size(); ++state_id) {
    getNextStatesAtState(state, statestate_space[state_id], graph, next_state_space[state_id]);
    size_next_state_space += sizeof(size_t) * next_state_space[state_id].size();
  }

  std::cout << "Size of next state cache: " << size_next_state_space << 
    std::endl;

  // 

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
  testStateSpacePopulation(graph);

  return 0;
}
