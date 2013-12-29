#include<fstream>
#include<cstdlib>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <rl_pursuit/planning/ValueIteration.h>

#include <rl_pursuit/common/Util.h>
#include <rl_pursuit/planning/MCTS.h>
#include <rl_pursuit/planning/UCTEstimator.h>
#include <rl_pursuit/planning/ModelUpdaterSingle.h>
#include <rl_pursuit/planning/IdentityStateMapping.h>

#include <bwi_guidance_solver/heuristic_solver_qrr14.h>
#include <bwi_guidance_solver/person_estimator_qrr14.h>
#include <bwi_guidance_solver/person_model_qrr14.h>
#include <bwi_guidance_solver/utils.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_mapper/map_utils.h>

#define MAX_ROBOTS 5

#ifdef EVALUATE_DEBUG
#define EVALUATE_OUTPUT(x) std::cout << x << std::endl
#else
#define EVALUATE_OUTPUT(x) ((void) 0)
#endif

using namespace bwi_guidance;

URGenPtr rng;
boost::shared_ptr<RNG> mcts_rng;
UCTEstimator<StateQRR14, ActionQRR14>::Params mcts_estimator_params;
MCTS<StateQRR14, ActionQRR14>::Params mcts_params;

/* Parameters with default values */
std::string data_directory = "";
std::string vi_policy_file = "vi.txt";
std::string model_file = "model.txt";
std::string results_file = "result.txt";
std::string map_file = "";
std::string graph_file = "";
int seed = 12345;
int num_instances = 10;
int runs_per_instance = 1;
float distance_limit = 300.f;
bool allow_robot_current_idx = false;
float visibility_range = 0.0f;
bool allow_goal_visibility = false;
bool mcts_enabled = false;

/* if mcts is enabled, bump num_methods to MAX_METHODS */
enum Method {
  HEURISTIC = 0,
  VI = 1,
  MCTS_METHOD = 2,
  MAX_METHODS = 3
};
int num_methods = 2;

struct InstanceResult {
  float avg_distance[MAX_METHODS][MAX_ROBOTS];
  float percent_completion[MAX_METHODS][MAX_ROBOTS];
  float true_vi_value[MAX_ROBOTS];
};

std::ostream& operator<< (std::ostream& stream, const InstanceResult& ir) {
  for (int i = 0; i < MAX_ROBOTS; ++i) {
    for (int j = 0; j < num_methods; ++j) {
      stream << ir.avg_distance[j][i] / ir.true_vi_value[MAX_ROBOTS - 1];
      if (i != MAX_ROBOTS - 1 || j != num_methods -1) {
        stream << ",";
      }
    }
  }
  return stream;
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

  /* Method 1 - Heuristic Solver */
  HeuristicSolver hs(map, graph, goal_idx, allow_robot_current_idx,
      pixel_visibility_range, allow_goal_visibility); 

  /* Method 2 - Value Iteration */
  boost::shared_ptr<PersonModelQRR14> model(
      new PersonModelQRR14(graph, map, goal_idx, indexed_model_file, 
        allow_robot_current_idx, pixel_visibility_range,
        allow_goal_visibility));
  boost::shared_ptr<PersonEstimatorQRR14> estimator(new PersonEstimatorQRR14);
  float epsilon = 0.05f / map.info.resolution;
  float delta = -500.0f / map.info.resolution;

  ValueIteration<StateQRR14, ActionQRR14> vi (model, estimator, 1.0, epsilon,
      1000, 0.0f, delta);

  /* Method 3 - MCTS - may or may not be enabled */
  boost::shared_ptr<MCTS<StateQRR14, ActionQRR14> > mcts;
  if (mcts_enabled) {
    model->initializeRNG(rng); 
    boost::shared_ptr<ModelUpdaterSingle<StateQRR14, ActionQRR14> >
      mcts_model_updator(
          new ModelUpdaterSingle<StateQRR14, ActionQRR14>(model));
    boost::shared_ptr<IdentityStateMapping<StateQRR14> > mcts_state_mapping(
        new IdentityStateMapping<StateQRR14>);
    boost::shared_ptr<UCTEstimator<StateQRR14, ActionQRR14> > uct_estimator(
        new UCTEstimator<StateQRR14, ActionQRR14>( mcts_rng,
          mcts_estimator_params));
    mcts.reset(new MCTS<StateQRR14, ActionQRR14>(uct_estimator,
          mcts_model_updator, mcts_state_mapping, mcts_params));
  }

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

  for (int method = 0; method < num_methods; ++method) {
    for (int starting_robots = 1; starting_robots <= 5; ++starting_robots) {

      EVALUATE_OUTPUT("Testing method " << method << " with " << 
          starting_robots << " robots.");

      float sum_instance_distance = 0;
      int count_successful = 0;
      float true_distance = 0;

      for (int run = 0; run < runs_per_instance; ++run) {

        StateQRR14 current_state; 
        current_state.graph_id = start_idx;
        current_state.direction = start_direction;
        current_state.num_robots_left = starting_robots;
        current_state.robot_direction = NONE;
        current_state.visible_robot = NONE;
        true_distance = -estimator->getValue(current_state);
        float instance_distance = 0;

        EVALUATE_OUTPUT(" - start " << current_state);

        if (method == MCTS_METHOD) {
          mcts->restart();
          for (int i = 0; i < 10; ++i) {
            mcts->search(current_state); // Initial search for 10 seconds
          }
        }

        float reward = 0;
        float distance_limit_pxl = 
          ((float)distance_limit) / map.info.resolution;

        while (current_state.graph_id != goal_idx && instance_distance <=
            distance_limit_pxl) {

          std::vector<StateQRR14> next_states;
          std::vector<float> probabilities;
          std::vector<float> rewards;

          // Deterministic system transitions
          while (true) {

            ActionQRR14 action;
            if (method == VI) {
              action = vi.getBestAction(current_state);
            } else if (method == HEURISTIC) {
              action = hs.getBestAction(current_state);
            } else if (method == MCTS_METHOD) {
              action = mcts->selectWorldAction(current_state);
            }
            EVALUATE_OUTPUT("   action: " << action);

            model->getTransitionDynamics(current_state, action, next_states, 
                rewards, probabilities);

            if (action.type == DO_NOTHING) {
              // Manual transition
              break;
            }

            // The human does not move for this action, and a single next state
            // is present
            current_state = next_states[0];
            if (method == MCTS_METHOD) {
              std::cout << " - performing single MCTS search (1s)" << std::endl;
              mcts->search(current_state);
            }
            EVALUATE_OUTPUT(" - auto " << current_state);
          }

          // Select next state choice based on probabilities
          int choice = select(probabilities, rng);
          StateQRR14 old_state = current_state;
          current_state = next_states[choice];
          float transition_distance =
            bwi_mapper::getEuclideanDistance(old_state.graph_id,
                current_state.graph_id, graph);
          instance_distance += transition_distance;

          // Perform an MCTS search after next state is decided (not perfect)
          // Only perform search if system is left with any future action choice
          if (!model->isTerminalState(current_state) &&
              (current_state.num_robots_left != 0 ||
               current_state.visible_robot != NONE)) {
            if (method == MCTS_METHOD) {
              int distance = transition_distance * map.info.resolution;
              EVALUATE_OUTPUT(" - performing MCTS search for " << distance <<
                  " seconds");
              std::cout << " - performing MCTS search for " << distance <<
                " seconds" << std::endl;
              for (int i = 0; i < distance; ++i) {
                mcts->search(current_state);
              }
            }
          }

          EVALUATE_OUTPUT(" - manual " << current_state);
          reward += rewards[choice];
        }

        if (current_state.graph_id == goal_idx) {
          ++count_successful;
        }
        sum_instance_distance += instance_distance;
      }

      if (method == VI) {
        result.true_vi_value[starting_robots - 1] = true_distance;
        result.true_vi_value[starting_robots - 1] *= map.info.resolution;
      } 

      result.avg_distance[method][starting_robots - 1] = 
        sum_instance_distance / runs_per_instance;
      result.avg_distance[method][starting_robots - 1] *= map.info.resolution;
      result.percent_completion[method][starting_robots - 1] =
        ((float) count_successful * 100.0f) / runs_per_instance;

    }
  }

  return result;

}

int processOptions(int argc, char** argv) {

  std::string mcts_params_file, mcts_estimator_params_file;
  std::string appName = boost::filesystem::basename(argv[0]); 
  std::vector<std::string> sentence; 

  /** Define and parse the program options 
  */ 
  namespace po = boost::program_options; 
  po::options_description desc("Options"); 
  desc.add_options() 
    ("map-file,M", po::value<std::string>(&map_file)->required(), 
     "YAML map file") 
    ("graph-file,G", po::value<std::string>(&graph_file)->required(), 
     "YAML graph file corresponding to the map") 
    ("mcts-params", po::value<std::string>(&mcts_params_file), 
     "JSON MCTS Parameter File") 
    ("mcts-estimator-params", 
     po::value<std::string>(&mcts_estimator_params_file), 
     "JSON MCTS Estimator Parameter File") 
    ("data-directory,D", po::value<std::string>(&data_directory), 
     "Data directory (defaults to runtime directory)") 
    ("allow-robot-current,a", "Allow robot to be placed at current index") 
    ("allow-goal-visibility,V", "Allow goal visibility to affect human model")
    ("seed,s", po::value<int>(&seed), "Random seed")  
    ("num-instances,n", po::value<int>(&num_instances), "Number of Instances") 
    ("runs-per-instance,r", po::value<int>(&runs_per_instance), 
     "Averge each instance over these many runs") 
    ("visibility-range,r", po::value<float>(&visibility_range), 
     "Simulator visibility range in meters.")
    ("distance-limit,d", po::value<float>(&distance_limit), 
     "Max distance at which to terminate episode."); 

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

  if (!mcts_params_file.empty() && !mcts_estimator_params_file.empty()) {
    Json::Value mcts_json, mcts_estimator_json;
    if (!readJson(mcts_params_file, mcts_json)) {
      return -1;
    }
    mcts_params.fromJson(mcts_json);
    if (!readJson(mcts_estimator_params_file, mcts_estimator_json)) {
      return -1;
    }
    mcts_estimator_params.fromJson(mcts_estimator_json);
    mcts_enabled = true;
    num_methods = MAX_METHODS;
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
  std::cout << "Allowing robot at current idx: " << allow_robot_current_idx <<
    std::endl;
  boost::mt19937 mt(seed);
  boost::uniform_real<float> u(0.0f, 1.0f);
  rng.reset(new URGen(mt, u));

  mcts_rng.reset(new RNG(seed));

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
    InstanceResult res = 
      testInstance(graph, map, start_idx, start_direction, goal_idx);
    std::cout << "  ... Done" << std::endl;

    // std::cout << res << std::endl;
    fout << res << std::endl;
  }
  fout.close();

  return 0;
}
