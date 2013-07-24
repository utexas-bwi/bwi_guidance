#include<fstream>

#include <bwi_exp1_solver/ValueIteration.h>
#include <bwi_exp1_solver/person_estimator.h>
#include <bwi_exp1_solver/person_model.h>
#include <topological_mapper/map_loader.h>

using namespace bwi_exp1;

void testValueIteration(topological_mapper::Graph& graph, const std::string& file = "") {

  size_t goal_idx = 33;

  boost::shared_ptr<PersonModel> model(new PersonModel(graph, goal_idx));
  boost::shared_ptr<PersonEstimator> estimator(
      new PersonEstimator(model->getStateSpaceSize(), 0));
  ValueIteration<state_t, action_t> vi(model, estimator, 0.98, 1000);

  bool policyAvailable = false;
  if (!file.empty()) {
    std::ifstream my_file(file.c_str());
    if (my_file.good()) {
      policyAvailable = true;
    }
  }
  if (policyAvailable) {
    vi.loadPolicy(file);
    std::cout << "Read policy from file: " << file << std::endl;
  } else {
    vi.computePolicy();
    vi.savePolicy("policy.txt");
    std::cout << "Saved policy to file: policy.txt" << std::endl;
  }

  // Now perform a max walk from start state to goal state
  while(true) {
    size_t start_idx, starting_robots, start_direction;
    std::cout << "Enter start idx: ";
    std::cin >> start_idx;
    std::cout << "Enter start direction: ";
    std::cin >> start_direction;
    std::cout << "Enter robots remaining: ";
    std::cin >> starting_robots;
    state_t current_state_idx = 
      model->canonicalizeState(start_idx, start_direction, starting_robots);
    State current_state = 
      model->resolveState(current_state_idx); 
    while (current_state.graph_id != goal_idx) {
      std::cout << "At State (" << current_state.graph_id << ", " << current_state.direction << ", " << current_state.num_robots_left << ")" << std::endl;
      std::cout << "Value of this state: " << estimator->getValue(current_state_idx) << std::endl;
      action_t action_idx = vi.getBestAction(current_state_idx);
      Action action = model->resolveAction(current_state_idx, action_idx);
      if (action.type == PLACE_ROBOT) {
        std::cout << "FOUND ROBOT. Robot points towards " << action.graph_id << std::endl;
      }
      std::vector<state_t> next_states;
      std::vector<float> probabilities;
      std::vector<float> rewards;
      model->getTransitionDynamics(current_state_idx, action_idx, next_states, 
          rewards, probabilities);

      for (size_t next_state_counter = 0; next_state_counter < next_states.size(); ++next_state_counter) {
        state_t next_state_idx = next_states[next_state_counter];
        State next_state = model->resolveState(next_state_idx);
        std::cout << "  - #" << next_state_counter << " Leads to State (" << next_state.graph_id << ", " << next_state.direction << ", " << next_state.num_robots_left << ") with probability " << probabilities[next_state_counter] << std::endl;
      }
      std::cout << "Choice: ";
      int choice;
      std::cin >> choice;
      current_state_idx = next_states[choice];
      current_state = model->resolveState(current_state_idx);
    }
    std::cout << "VALUE of final state: " << estimator->getValue(current_state_idx) << std::endl;
  }

}

// void testActionChoices(topological_mapper::Graph& graph) {
// 
//   State state;
//   state.graph_id = 16;
//   state.num_robots_left = 1;
//   std::vector<Action> actions;
//   getActionsAtState(state, graph, actions);
// 
//   std::cout << "At State (" << state.graph_id << ", " << state.direction << ", " << state.num_robots_left << ")" << std::endl;
//   for (size_t action_counter = 0; action_counter < actions.size(); ++action_counter) {
//     Action& action = actions[action_counter];
//     std::cout << "  - An action is (" << action.type << ", " << action.graph_id << ")" << std::endl;
//   }
// 
// }
// 
// void testNextDirectionComputation(topological_mapper::Graph& graph) {
// 
//   size_t direction = 12;
//   size_t current_state = 78;
//   size_t next_state = 79;
//   size_t next_direction;
// 
//   next_direction = bwi_exp1::computeNextDirection(direction, current_state, next_state, graph);
//   std::cout << "(" << current_state << ", " << direction << ") -> " <<
//       "(" << next_state << ", " << next_direction << ")" << std::endl;
// 
//   current_state = next_state;
//   direction = next_direction;
//   next_state = 65;
//   next_direction = bwi_exp1::computeNextDirection(direction, current_state, next_state, graph);
//   std::cout << "(" << current_state << ", " << direction << ") -> " <<
//       "(" << next_state << ", " << next_direction << ")" << std::endl;
// 
//   current_state = next_state;
//   direction = next_direction;
//   next_state = 57;
//   next_direction = bwi_exp1::computeNextDirection(direction, current_state, next_state, graph);
//   std::cout << "(" << current_state << ", " << direction << ") -> " <<
//       "(" << next_state << ", " << next_direction << ")" << std::endl;
// 
//   current_state = next_state;
//   direction = next_direction;
//   next_state = 55;
//   next_direction = bwi_exp1::computeNextDirection(direction, current_state, next_state, graph);
//   std::cout << "(" << current_state << ", " << direction << ") -> " <<
//       "(" << next_state << ", " << next_direction << ")" << std::endl;
// 
//   current_state = next_state;
//   direction = next_direction;
//   next_state = 51;
//   next_direction = bwi_exp1::computeNextDirection(direction, current_state, next_state, graph);
//   std::cout << "(" << current_state << ", " << direction << ") -> " <<
//       "(" << next_state << ", " << next_direction << ")" << std::endl;
// 
//   current_state = next_state;
//   direction = next_direction;
//   next_state = 33;
//   next_direction = bwi_exp1::computeNextDirection(direction, current_state, next_state, graph);
//   std::cout << "(" << current_state << ", " << direction << ") -> " <<
//       "(" << next_state << ", " << next_direction << ")" << std::endl;
// 
// }

int main(int argc, char** argv) {

  if (argc < 3) {
    std::cerr << "USAGE: " << argv[0] 
        << " <yaml-map-file> <yaml-graph-file> [<saved policy>]" << std::endl;
    return -1;
  }

  topological_mapper::MapLoader mapper(argv[1]);
  topological_mapper::Graph graph;
  nav_msgs::MapMetaData info;
  mapper.getMapInfo(info);
  topological_mapper::readGraphFromFile(argv[2], info, graph);

  //testNextDirectionComputation(graph);
  //testActionChoices(graph);
  if (argc >= 4) {
    testValueIteration(graph, std::string(argv[3]));
  } else {
    testValueIteration(graph);
  }

  return 0;
}
