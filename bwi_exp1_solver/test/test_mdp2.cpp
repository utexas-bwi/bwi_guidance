#include<fstream>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <bwi_exp1_solver/ValueIteration.h>
#include <bwi_exp1_solver/heuristic_solver.h>
#include <bwi_exp1_solver/person_estimator2.h>
#include <bwi_exp1_solver/person_model2.h>
#include <topological_mapper/map_loader.h>
#include <topological_mapper/map_utils.h>

using namespace bwi_exp1;

std::string vi_policy_file = "vi.txt";
std::string model_file = "model.txt";
std::string map_file = "";
std::string graph_file = "";
bool use_heuristic = false;
bool use_vi = false; 
bool allow_robot_current_idx = false;
int goal_idx = 46;

void test(topological_mapper::Graph& graph, 
    nav_msgs::OccupancyGrid& map) {

  std::string indexed_model_file = boost::lexical_cast<std::string>(goal_idx)
    + "_" + model_file;
  std::string indexed_vi_file = boost::lexical_cast<std::string>(goal_idx)
    + "_" + vi_policy_file;

  boost::shared_ptr<PersonModel2> model(
      new PersonModel2(graph, map, goal_idx, indexed_model_file, 
        allow_robot_current_idx));
  boost::shared_ptr<PersonEstimator2> estimator(new PersonEstimator2);
  ValueIteration<State2, Action> vi(model, estimator, 
      1.0, 1.0, 1000, 0.0, -10000.0);
  HeuristicSolver hi(map, graph, goal_idx, allow_robot_current_idx); 

  std::ifstream vi_ifs(indexed_vi_file.c_str());
  if (vi_ifs.good()) {
    vi.loadPolicy(indexed_vi_file);
    std::cout << "Read policy from file: " << vi_policy_file << std::endl;
  } else {
    vi.computePolicy();
    vi.savePolicy(indexed_vi_file);
    std::cout << "Computed and saved policy to file: " 
      << indexed_vi_file << std::endl;
  }

  // Now perform a max walk from start state to goal state
  while(true) {

    int start_idx, starting_robots, start_direction, start_crs, start_vrl;
    std::cout << "Enter start currentGraphIdx: ";
    std::cin >> start_idx;
    std::cout << "Enter start currentDirection: ";
    std::cin >> start_direction;
    std::cout << "Enter start numRobotsLeft: ";
    std::cin >> starting_robots;
    std::cout << "Enter start currentRobotStatus " 
      << "(NO_ROBOT=-1, DIR_UNASSIGNED=-2): ";
    std::cin >> start_crs;
    std::cout << "Enter start visibleRobotLocation: ";
    std::cin >> start_vrl;

    State2 current_state; 
    current_state.graph_id = start_idx;
    current_state.direction = start_direction;
    current_state.num_robots_left = starting_robots;
    current_state.current_robot_status = start_crs;
    current_state.visible_robot_location = start_vrl;

    while (current_state.graph_id != goal_idx) {

      std::vector<State2> next_states;
      std::vector<float> probabilities;
      std::vector<float> rewards;

      while (true) {

        std::cout << "At State " << current_state << std::endl; 
        std::cout << "Value of this state (from VI): " << 
          estimator->getValue(current_state) << std::endl;

        std::vector<Action> actions;
        int count = 0;
        model->getActionsAtState(current_state, actions);
        std::string str[3];
        str[DO_NOTHING] = "DO_NOTHING";
        str[DIRECT_PERSON] = "DIRECT_PERSON";
        str[PLACE_ROBOT] = "PLACE_ROBOT";
        BOOST_FOREACH(const Action& action, actions) {
          std::cout << "#" << count << " Action: " << str[action.type] << " " << action.graph_id << std::endl;
          model->getTransitionDynamics(current_state, action, next_states, rewards, probabilities);
          // for (size_t next_state_counter = 0; next_state_counter < next_states.size(); ++next_state_counter) {
          //   State2& next_state = next_states[next_state_counter];
          //   std::cout << "  - #" << next_state_counter << " Leads to State " << next_state << " with probability " << probabilities[next_state_counter] << " and reward " << rewards[next_state_counter] << std::endl;
          // }
          ++count;
        }
        Action vi_action = vi.getBestAction(current_state);
        Action hi_action = hi.getBestAction(current_state);
        Action action;
        if (use_heuristic) {
          action = hi_action;
        } else if (use_vi) {
          action = vi_action;
        } else {
          std::cout << "VI picks: " << str[vi_action.type] << " " << vi_action.graph_id << std::endl;
          std::cout << "Heuristic picks: " << str[hi_action.type] << " " << hi_action.graph_id << std::endl;
          std::cout << "Choice: ";
          int choice;
          std::cin >> choice;
          action = actions[choice];
        }
        std::cout << "Selected: " << str[action.type] << " " << action.graph_id << std::endl;

        model->getTransitionDynamics(current_state, action, next_states, 
            rewards, probabilities);

        if (action.type == DO_NOTHING) {
          // Manual transition
          break;
        }

        // The human does not move for this action, and a single next state is present
        if (action.type == DIRECT_PERSON) {
          std::cout << "Robot points towards " << action.graph_id << std::endl;
        } else {
          std::cout << "See robot at " << action.graph_id << std::endl;
        }
        current_state = next_states[0];
        std::cout << " - AUTO transition to " << current_state << std::endl;

      }

      std::cout << "Waiting for MANUAL transition..." << std::endl;

      for (size_t next_state_counter = 0; next_state_counter < next_states.size(); ++next_state_counter) {
        State2& next_state = next_states[next_state_counter];
        std::cout << "  - #" << next_state_counter << " Leads to State " << next_state << " with probability " << probabilities[next_state_counter] << std::endl;
      }
      std::cout << "Choice: ";
      int choice;
      std::cin >> choice;
      current_state = next_states[choice];
    }
    std::cout << "VALUE of final state (from VI): " << estimator->getValue(current_state) << std::endl;
  }

}

int processOptions(int argc, char** argv) {

  std::string appName = boost::filesystem::basename(argv[0]); 
  std::vector<std::string> sentence; 

  /** Define and parse the program options 
  */ 
  namespace po = boost::program_options; 
  po::options_description desc("Options"); 
  desc.add_options() 
    ("heuristic,h", "Use heuristic to auto-select best option") 
    ("vi,v", "Use VI to auto-select best option") 
    ("map_file,M", po::value<std::string>(&map_file)->required(), "YAML map file") 
    ("graph_file,G", po::value<std::string>(&graph_file)->required(), "YAML graph file") 
    ("allow_robot_current,a", "Allow robot to be placed at current index") 
    ("goal_idx,g", po::value<int>(&goal_idx), "Goal index location"); 

  po::positional_options_description positionalOptions; 
  positionalOptions.add("map_file", 1); 
  positionalOptions.add("graph_file", 1); 

  po::variables_map vm; 

  try { 
    po::store(po::command_line_parser(argc, argv).options(desc) 
        .positional(positionalOptions).run(), 
        vm); // throws on error 

    po::notify(vm); // throws on error, so do after help in case 
    // there are any problems 
  } catch(boost::program_options::required_option& e) { 
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl; 
    std::cout << desc << std::endl;
    return -1; 
  } catch(boost::program_options::error& e) { 
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl; 
    std::cout << desc << std::endl;
    return -1; 
  } 

  // can do this without fear because it is required to be present 
  if (vm.count("heuristic")) { 
    use_heuristic = true;
  } else if (vm.count("vi")) {
    use_vi = true;
  }

  if (vm.count("allow_robot_current")) {
    allow_robot_current_idx = true;
  }

  return 0;
}

int main(int argc, char** argv) {

  int ret = processOptions(argc, argv);
  if (ret != 0) {
    return ret;
  }

  topological_mapper::MapLoader mapper(argv[1]);
  topological_mapper::Graph graph;
  nav_msgs::OccupancyGrid map;
  mapper.getMap(map);
  topological_mapper::readGraphFromFile(argv[2], map.info, graph);

  test(graph, map);

  return 0;
}
