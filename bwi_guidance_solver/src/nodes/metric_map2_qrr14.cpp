#include<fstream>
#include<cstdlib>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <bwi_rl/planning/ValueIteration.h>
#include <bwi_guidance_solver/heuristic_solver_qrr14.h>
#include <bwi_guidance_solver/person_estimator_qrr14.h>
#include <bwi_guidance_solver/person_model_qrr14.h>
#include <bwi_guidance_solver/utils.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_mapper/map_utils.h>

using namespace bwi_guidance;

URGenPtr rng;

std::string data_directory = "";
std::string vi_policy_file = "vi.txt";
std::string model_file = "model.txt";
std::string results_file = "metric_map2.txt";
std::string map_file = "";
std::string graph_file = "";
int seed = 12345;
int num_instances = 10;
int runs_per_instance = 1;
float distance_limit = 300.f;
bool allow_robot_current_idx = false;
float visibility_range = 0.0f;
bool allow_goal_visibility = false;

struct InstanceResult {
  float avg_hi_distance[15];
  float percent_hi_completion[15];
  float true_vi_value;
};

std::ostream& operator<< (std::ostream& stream, const InstanceResult& ir) {
  stream << ir.true_vi_value << ",";
  for (int i = 0; i < 15; ++i) {
    stream << ir.avg_hi_distance[i];
    if (i!=14) 
      stream << ",";
  }
  return stream;
}

int select(std::vector<float>& probabilities) {
  float random_value = (*rng)();
  float prob_sum = probabilities[0];
  for (int i = 1; i < probabilities.size(); ++i) {
    if (random_value < prob_sum) return i - 1;
    prob_sum += probabilities[i];
  }
  return probabilities.size() - 1;
}

InstanceResult testInstance(bwi_mapper::Graph& graph, 
    nav_msgs::OccupancyGrid& map, int start_idx, int start_direction, 
    int goal_idx) {

  InstanceResult result;

  std::string indexed_model_file = data_directory 
    + boost::lexical_cast<std::string>(goal_idx)
    + "_" + model_file;
  std::string indexed_vi_file = data_directory
    + boost::lexical_cast<std::string>(goal_idx)
    + "_" + vi_policy_file;

  float pixel_visibility_range = visibility_range / map.info.resolution;
  boost::shared_ptr<PersonModelQRR14> model(
      new PersonModelQRR14(graph, map, goal_idx, indexed_model_file, 
        allow_robot_current_idx, pixel_visibility_range,
        allow_goal_visibility));
  boost::shared_ptr<PersonEstimatorQRR14> estimator(new PersonEstimatorQRR14);
  float epsilon = 0.05f / map.info.resolution;
  float delta = -500.0f / map.info.resolution;
  ValueIteration<StateQRR14, ActionQRR14> vi(model, estimator, 1.0, epsilon, 1000, 0.0f,
      delta);
  HeuristicSolver hi(map, graph, goal_idx, allow_robot_current_idx,
      pixel_visibility_range, allow_goal_visibility); 

  std::ifstream vi_ifs(indexed_vi_file.c_str());
  if (vi_ifs.good()) {
    vi.loadPolicy(indexed_vi_file);
    vi.savePolicy(indexed_vi_file);
  } else {
    vi.computePolicy();
    vi.savePolicy(indexed_vi_file);
    std::cout << "Computed and saved policy for " << goal_idx << " to file: " 
      << indexed_vi_file << std::endl;
  }

  StateQRR14 true_state;
  true_state.graph_id = start_idx;
  true_state.direction = start_direction;
  true_state.num_robots_left = 5;
  true_state.robot_direction = NONE;
  true_state.visible_robot = NONE;
  float true_distance = -estimator->getValue(true_state);
  result.true_vi_value = true_distance;
  result.true_vi_value *= map.info.resolution;

    for (int starting_robots = 1; starting_robots <= 15; ++starting_robots) {

      float sum_instance_distance = 0;
      int count_successful = 0;

      for (int run = 0; run < runs_per_instance; ++run) {

        StateQRR14 current_state; 
        current_state.graph_id = start_idx;
        current_state.direction = start_direction;
        current_state.num_robots_left = starting_robots;
        current_state.robot_direction = NONE;
        current_state.visible_robot = NONE;

        /* std::cout << " START " << current_state << std::endl; */

        float reward = 0;
        float reward_limit = -((float)distance_limit) / map.info.resolution;

        while (current_state.graph_id != goal_idx && reward >= reward_limit) {

          std::vector<StateQRR14> next_states;
          std::vector<float> probabilities;
          std::vector<float> rewards;

          int increase_robots = 0;
          // Deterministic system transitions
          while (true) {
            increase_robots = 0;

            ActionQRR14 action;
              action = hi.getBestAction(current_state);
            /* std::cout << "   action: " << action << std::endl; */

            if (current_state.num_robots_left > 3) {
              increase_robots = current_state.num_robots_left - 3;
              current_state.num_robots_left = 3;
            }
            model->getTransitionDynamics(current_state, action, next_states, 
                rewards, probabilities);

            if (action.type == DO_NOTHING) {
              // Manual transition
              break;
            } 

            // The human does not move for this action, and a single next state is present
            current_state = next_states[0];
            current_state.num_robots_left += increase_robots;
            /* std::cout << " - auto " << current_state << std::endl; */
          }

          // Select next state choice based on probabilities
          int choice = select(probabilities);
          current_state = next_states[choice];
          current_state.num_robots_left += increase_robots;
          /* std::cout << " - manual " << current_state << std::endl; */
          reward += rewards[choice];
        }

        if (current_state.graph_id == goal_idx) {
          ++count_successful;
        }
        sum_instance_distance += -reward;
      }

        result.avg_hi_distance[starting_robots - 1] = 
          sum_instance_distance / runs_per_instance;
        result.avg_hi_distance[starting_robots - 1] *= map.info.resolution;
        result.percent_hi_completion[starting_robots - 1] =
          ((float) count_successful * 100.0f) / runs_per_instance;

    }

  return result;

}

int processOptions(int argc, char** argv) {

  std::string appName = boost::filesystem::basename(argv[0]); 
  std::vector<std::string> sentence; 

  /** Define and parse the program options 
  */ 
  namespace po = boost::program_options; 
  po::options_description desc("Options"); 
  desc.add_options() 
    ("map-file,M", po::value<std::string>(&map_file)->required(), "YAML map file") 
    ("graph-file,G", po::value<std::string>(&graph_file)->required(), "YAML graph file") 
    ("data-directory,D", po::value<std::string>(&data_directory), "Data directory (defaults to runtime directory)") 
    ("allow-robot-current,a", "Allow robot to be placed at current index") 
    ("seed,s", po::value<int>(&seed), "Random seed")  
    ("num-instances,n", po::value<int>(&num_instances), "Number of Instances") 
    ("runs-per-instance,r", po::value<int>(&runs_per_instance), "Averge each instance over these many runs") 
    ("allow-goal-visibility,V", "Allow goal visibility to affect human model")
    ("visibility-range,r", po::value<float>(&visibility_range), "Simulator visibility")
    ("distance-limit,d", po::value<float>(&distance_limit), "Max distance at which to terminate episode"); 

  // po::positional_options_description positionalOptions; 
  // positionalOptions.add("map-file", 1); 
  // positionalOptions.add("graph-file", 1); 

  po::variables_map vm; 

  try { 
    po::store(po::command_line_parser(argc, argv).options(desc) 
        /* .positional(positionalOptions).allow_unregistered().run(),  */
        .allow_unregistered().run(), 
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

  if (vm.count("allow-robot-current")) {
    allow_robot_current_idx = true;
  }
  if (vm.count("allow-goal-visibility")) {
    allow_goal_visibility = true;
  }

  return 0;
}

int main(int argc, char** argv) {

  int ret = processOptions(argc, argv);
  if (ret != 0) {
    return ret;
  }

  std::cout << "Using random seed: " << seed << std::endl;
  std::cout << "Number of instances: " << num_instances << std::endl;
  std::cout << "Allowing robot at current idx: " << allow_robot_current_idx << std::endl;
  boost::mt19937 mt(seed);
  boost::uniform_real<float> u(0.0f, 1.0f);
  rng.reset(new URGen(mt, u));

  bwi_mapper::MapLoader mapper(map_file);
  bwi_mapper::Graph graph;
  nav_msgs::OccupancyGrid map;
  mapper.getMap(map);
  bwi_mapper::readGraphFromFile(graph_file, map.info, graph);

  boost::uniform_int<int> idx_dist(0, boost::num_vertices(graph) - 1);
  UIGen idx_gen(mt, idx_dist);
  boost::uniform_int<int> direction_dist(0, 15);
  UIGen direction_gen(mt, direction_dist);
  
  std::ofstream fout((data_directory + results_file).c_str());
  for (int i = 0; i < num_instances; ++i) {
    int start_idx = idx_gen();
    int goal_idx = idx_gen();
    while (goal_idx == start_idx) {
      goal_idx = idx_gen();
    }
    int start_direction = direction_gen();
    std::cout << "#" << i << " Testing [" << start_idx << "," <<
      start_direction << "," << goal_idx << "]: " << std::endl;
    InstanceResult res = testInstance(graph, map, start_idx, start_direction, goal_idx);
    std::cout << res << std::endl;
    fout << i << "," << start_idx << "," << start_direction << "," << goal_idx
      << "," << res << std::endl;
  }
  fout.close();

  return 0;
}
