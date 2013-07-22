#include <bwi_exp1_solver/simple_model.h>
#include <bwi_exp1_solver/value_iteration.h>
#include <topological_mapper/map_loader.h>

using namespace bwi_exp1;

void testValueIteration(topological_mapper::Graph& graph) {

  int num_vertices = boost::num_vertices(graph);
  PersonModel model(graph, 33);
  std::vector<float> featmax(3, 0.0), featmin(3, 0.0);
  featmax[0] = (float) (num_vertices - 1);
  featmax[1] = (float) (15);
  featmax[2] = (float) (5);
  std::vector<int> statesPerDim(3, 0);
  statesPerDim[0] = num_vertices - 1;
  statesPerDim[1] = 16 - 1;
  statesPerDim[2] = 6 - 1;

  ValueIteration2 vi(30, 0.98, 1000, 3600.0, 1, featmax, featmin, statesPerDim); 
  vi.setModel((MDPModel*) &model);
  vi.initStates();
  vi.planOnNewModel();

// 
//   // Now perform a max walk from start state to goal state
//   while(true) {
//     size_t start_idx, starting_robots, start_direction;
//     std::cout << "Enter start idx: ";
//     std::cin >> start_idx;
//     std::cout << "Enter start direction: ";
//     std::cin >> start_direction;
//     std::cout << "Enter robots remaining: ";
//     std::cin >> starting_robots;
//     size_t current_state_idx = constructStateIndex(start_idx, start_direction, starting_robots);
//     while (state_space[current_state_idx].graph_id != goal_idx) {
//       State& current_state = state_space[current_state_idx];
//       std::cout << "At State (" << current_state.graph_id << ", " << current_state.direction << ", " << current_state.num_robots_left << ")" << std::endl;
//       std::cout << "Value of this state: " << value_space[current_state_idx] << std::endl;
//       Action& action = best_action_space[current_state_idx];
//       if (action.type == PLACE_ROBOT) {
//         std::cout << "FOUND ROBOT. Robot points towards " << action.graph_id << std::endl;
//       }
// 
//       // std::cout << " Actions available: " << std::endl;
//       // std::vector<Action>& actions = action_space[current_state.graph_id * 2 + (current_state.num_robots_left != 0)];
//       // for (size_t action_idx = 0; action_idx < actions.size(); ++action_idx) {
//       //   Action action = actions[action_idx];
//       //   std::cout << "  - #" << action_idx << " (" << action.type << ", " << action.graph_id << ")" << std::endl;
//       // }
//       // std::cout << "Choice: ";
//       // int choice;
//       // std::cin >> choice;
//       // Action& action = actions[choice];
//       std::vector<size_t> next_states;
//       std::vector<float> probabilities;
//       getTransitionProbabilities(current_state, state_space, graph, action, next_states, probabilities);
// 
//       for (size_t next_state_counter = 0; next_state_counter < next_states.size(); ++next_state_counter) {
//         size_t next_state_idx = next_states[next_state_counter];
//         State& next_state = state_space[next_state_idx];
//         std::cout << "  - #" << next_state_counter << " Leads to State (" << next_state.graph_id << ", " << next_state.direction << ", " << next_state.num_robots_left << ") with probability " << probabilities[next_state_counter] << std::endl;
//       }
//       std::cout << "Choice: ";
//       int choice;
//       std::cin >> choice;
//       current_state_idx = next_states[choice];
//     }
//     std::cout << "VALUE of final state: " << value_space[current_state_idx] << std::endl;
//   }
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

  testValueIteration(graph);

  return 0;
}
