#include<fstream>

#include <bwi_exp1_solver/ValueIteration.h>
#include <bwi_exp1_solver/person_estimator2.h>
#include <bwi_exp1_solver/person_model2.h>
#include <topological_mapper/map_loader.h>
#include <topological_mapper/map_utils.h>
#include <boost/foreach.hpp>

using namespace bwi_exp1;

void testValueIteration(topological_mapper::Graph& graph, 
    nav_msgs::OccupancyGrid& map, const std::string& file = "") {

  size_t goal_idx = 22;

  boost::shared_ptr<PersonModel2> model(new PersonModel2(graph, map, goal_idx));
  boost::shared_ptr<PersonEstimator2> estimator(new PersonEstimator2);
  ValueIteration<State2, Action> vi(model, estimator, 1.0, 1.0, 1000);

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
    State2 current_state; 
    current_state.graph_id = start_idx;
    current_state.direction = start_direction;
    current_state.num_robots_left = starting_robots;
    current_state.current_robot_direction = NO_ROBOT;
    current_state.next_robot_location = NO_ROBOT;
    while (current_state.graph_id != goal_idx) {

      std::cout << "At State " << current_state << std::endl; 
      std::cout << "Value of this state: " << estimator->getValue(current_state) << std::endl;

      Action action = vi.getBestAction(current_state);
      std::vector<State2> next_states;
      std::vector<float> probabilities;
      std::vector<float> rewards;
      model->getTransitionDynamics(current_state, action, next_states, 
          rewards, probabilities);

      while (action.type != DO_NOTHING) {
        // The human does not move for this action, and a single next state is present
        if (action.type == PLACE_ROBOT) {
          std::cout << "Robot points towards " << action.graph_id << std::endl;
        } else {
          std::cout << "See robot at " << action.graph_id << std::endl;
        }
        current_state = next_states[0];
        std::cout << " - auto transition to " << current_state << std::endl;
        action = vi.getBestAction(current_state);
        model->getTransitionDynamics(current_state, action, next_states, 
            rewards, probabilities);
      }

      for (size_t next_state_counter = 0; next_state_counter < next_states.size(); ++next_state_counter) {
        State2& next_state = next_states[next_state_counter];
        std::cout << "  - #" << next_state_counter << " Leads to State " << next_state << " with probability " << probabilities[next_state_counter] << std::endl;
      }
      std::cout << "Choice: ";
      int choice;
      std::cin >> choice;
      current_state = next_states[choice];
    }
    std::cout << "VALUE of final state: " << estimator->getValue(current_state) << std::endl;
  }

}

void testModel2(topological_mapper::Graph& graph, const nav_msgs::OccupancyGrid &map) {
  PersonModel2 model(graph, map, 33);

  State2 s;
  s.graph_id = 54;
  s.direction = 0;
  s.num_robots_left = 0;
  s.current_robot_direction = 42;
  s.next_robot_location = -1;

  std::cout << "At state " << s << std::endl;
  std::vector<Action> actions;
  model.getActionsAtState(s, actions);

  std::string str[3];
  str[0] = "NOOP";
  str[1] = "DIRECT_ROBOT";
  str[2] = "PLACE_FUTURE_ROBOT";
  BOOST_FOREACH(const Action& a, actions) {
    std::cout << "ACTION: " << str[a.type] << " " << a.graph_id << std::endl;
    std::vector<State2> next_states;
    std::vector<float> probabilities;
    std::vector<float> rewards;
    model.getTransitionDynamics(s, a, next_states, rewards, probabilities);
    for (size_t next_state_counter = 0; next_state_counter < next_states.size(); ++next_state_counter) {
      State2& next_state = next_states[next_state_counter];
      std::cout << "  - #" << next_state_counter << " Leads to State " << next_state << " with probability " << probabilities[next_state_counter] << " and reward " << rewards[next_state_counter] << std::endl;
    }
  }

}

int main(int argc, char** argv) {

  if (argc < 3) {
    std::cerr << "USAGE: " << argv[0] 
        << " <yaml-map-file> <yaml-graph-file> [<saved policy>]" << std::endl;
    return -1;
  }

  topological_mapper::MapLoader mapper(argv[1]);
  topological_mapper::Graph graph;
  nav_msgs::OccupancyGrid map;
  mapper.getMap(map);
  topological_mapper::readGraphFromFile(argv[2], map.info, graph);

  if (argc >= 4) {
    testValueIteration(graph, map, std::string(argv[3]));
  } else {
    testValueIteration(graph, map);
  }
//  testModel2(graph, map);

  return 0;
}
