#include <bwi_exp1/mdp.h>
#include <topological_mapper/map_loader.h>

void testStateSpacePopulation(topological_mapper::Graph& graph) {

  std::vector<bwi_exp1::State> state_space;
  bwi_exp1::populateStateSpace(graph, state_space);



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
