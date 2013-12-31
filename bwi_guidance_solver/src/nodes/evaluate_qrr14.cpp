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

/* Constants */
std::string VI_POLICY_FILE_SUFFIX = "vi";
std::string MODEL_FILE_SUFFIX = "model";
std::string DISTANCE_FILE_SUFFIX = "distance.txt";
std::string REWARD_FILE_SUFFIX = "reward.txt";

/* Parameters (with their defaults) */
std::string data_directory_ = "";
std::string map_file_ = "";
std::string graph_file_ = "";
int seed_ = 12345;
int num_instances_ = 1000;
float distance_limit_ = 300.f;
bool allow_robot_current_idx_ = false;
float visibility_range_ = 0.0f; // Infinite visibility
bool allow_goal_visibility_ = false;
MCTS<StateQRR14, ActionQRR14>::Params mcts_params_;
bool mcts_enabled_ = false;
int precompute_vi_ = -1;

/* Structures used to define a single method */
enum MethodType {
  HEURISTIC = 0,
  VI = 1,
  MCTS_TYPE = 2
};

struct MethodResult {
  float reward[MAX_ROBOTS];
  float distance[MAX_ROBOTS];
};

namespace Method {
#define PARAMS(_) \
  _(int,type,type,MCTS_TYPE) \
  _(float,gamma,gamma,1.0) \
  _(bool,use_intrinsic_reward,use_intrinsic_reward,false) \
  _(float,success_reward,success_reward,0.0) \
  _(float,mcts_initial_planning_time,mcts_initial_planning_time,10.0) \
  _(float,mcts_reward_bound,mcts_reward_bound,10000.0) 

  Params_STRUCT(PARAMS)
#undef PARAMS
}

std::vector<Method::Params> methods_;

/* Structures used to define a single problem instance */

struct InstanceResult {
  std::vector<MethodResult> results;
  std::vector<MethodResult> normalized_results;
};

/* Helper Functions */

// std::ostream& operator<< (std::ostream& stream, const InstanceResult& ir) {
//   for (int i = 0; i < MAX_ROBOTS; ++i) {
//     for (int j = 0; j < num_methods; ++j) {
//       stream << ir.avg_distance[j][i] / ir.true_vi_value[MAX_ROBOTS - 1];
//       if (i != MAX_ROBOTS - 1 || j != num_methods -1) {
//         stream << ",";
//       }
//     }
//   }
//   return stream;
// }

std::string getIndexedModelFile(int goal_idx) {
  return data_directory_ +
    boost::lexical_cast<std::string>(goal_idx) + "_" + MODEL_FILE_SUFFIX;
}

std::string getIndexedVIFile(int goal_idx, const Method::Params& params) {
  return data_directory_ +
    "goal" + boost::lexical_cast<std::string>(goal_idx) + "_" + 
    "gamma" + boost::lexical_cast<std::string>(params.gamma) + "_" + "intRw"
    + boost::lexical_cast<std::string>(params.use_intrinsic_reward) + "_" + 
    "successRw" + boost::lexical_cast<std::string>(params.success_reward) + 
    "_" + VI_POLICY_FILE_SUFFIX;
}

boost::shared_ptr<PersonModelQRR14> getModel(bwi_mapper::Graph& graph,
    nav_msgs::OccupancyGrid& map, int goal_idx) {

  std::string indexed_model_file = getIndexedModelFile(goal_idx);
  float pixel_visibility_range = visibility_range_ / map.info.resolution;

  boost::shared_ptr<PersonModelQRR14> model(
      new PersonModelQRR14(graph, map, goal_idx, indexed_model_file, 
        allow_robot_current_idx_, pixel_visibility_range,
        allow_goal_visibility_));
  return model;
}

boost::shared_ptr<ValueIteration<StateQRR14, ActionQRR14> > getVIInstance(
    nav_msgs::OccupancyGrid& map,
    const boost::shared_ptr<PersonModelQRR14>& model, 
    const boost::shared_ptr<PersonEstimatorQRR14>& estimator, int goal_idx, 
    const Method::Params& params) {

  float epsilon = 0.05f / map.info.resolution;
  float delta = -500.0f / map.info.resolution;
  int num_iterations = 1000;

  boost::shared_ptr<ValueIteration<StateQRR14, ActionQRR14> > vi(
      new ValueIteration<StateQRR14, ActionQRR14>(model, estimator, 
        params.gamma, epsilon, num_iterations, -delta, delta));

  std::string indexed_vi_file = getIndexedVIFile(goal_idx, params);
  std::ifstream vi_ifs(indexed_vi_file.c_str());
  if (vi_ifs.good()) {
    vi->loadPolicy(indexed_vi_file);
  } else {
    EVALUATE_OUTPUT("VI policy not found. Computing...");
    vi->computePolicy();
    vi->savePolicy(indexed_vi_file);
    EVALUATE_OUTPUT("Computed and saved policy for " << goal_idx << " to file: " 
      << indexed_vi_file);
  }
  vi_ifs.close();

  return vi;
}

float getDistanceNormalizationValue(bwi_mapper::Graph& graph, int goal_idx, 
    int start_idx) {
  std::vector<size_t> temp_path;
  return bwi_mapper::getShortestPathWithDistance(start_idx, goal_idx, temp_path,
      graph);
}

float getRewardNormalizationValue(
    const boost::shared_ptr<PersonEstimatorQRR14>& estimator,
    int start_idx, int start_direction) {
  StateQRR14 current_state;
  current_state.graph_id = start_idx;
  current_state.direction = start_direction;
  current_state.num_robots_left = MAX_ROBOTS;
  current_state.robot_direction = NONE;
  current_state.visible_robot = NONE;
  return estimator->getValue(current_state);
}

/* Top level execution functions */

void precomputeVI(bwi_mapper::Graph& graph, nav_msgs::OccupancyGrid& map,
    int goal_idx, const Method::Params& params) {
  boost::shared_ptr<PersonModelQRR14> model = getModel(graph, map, goal_idx);
  boost::shared_ptr<PersonEstimatorQRR14> estimator(new PersonEstimatorQRR14);
  getVIInstance(map, model, estimator, goal_idx, params);
}

InstanceResult testInstance(int instance_number, bwi_mapper::Graph& graph, 
    nav_msgs::OccupancyGrid& map, int start_idx, int start_direction, 
    int goal_idx, const std::vector<Method::Params>& methods) {

  InstanceResult result;
  float pixel_visibility_range = visibility_range_ / map.info.resolution;

  // We can create the model right now as loading the model is not dependent 
  // on the method parameters. We just need to make sure we reinitialize the
  // rng and update the reward structure for every method as necessary
  boost::shared_ptr<PersonModelQRR14> model = getModel(graph, map, goal_idx);
  boost::shared_ptr<PersonEstimatorQRR14> estimator;

  for (size_t method = 0; method < methods.size(); ++method) {

    MethodResult method_result;

    boost::shared_ptr<HeuristicSolver> hs;
    boost::shared_ptr<ValueIteration<StateQRR14, ActionQRR14> > vi;
    boost::shared_ptr<MCTS<StateQRR14, ActionQRR14> > mcts;

    const Method::Params& params = methods[method];
    model->updateRewardStructure(params.success_reward, 
        params.use_intrinsic_reward);

    if (params.type == HEURISTIC) {
      hs.reset(new HeuristicSolver(map, graph, goal_idx,
            allow_robot_current_idx_, pixel_visibility_range,
            allow_goal_visibility_)); 
    } else if (params.type == VI) {
      estimator.reset( new PersonEstimatorQRR14);
      vi = getVIInstance(map, model, estimator, goal_idx, params);
    } else if (params.type == MCTS_TYPE) {

      if (!mcts_enabled_) {
        throw std::runtime_error(
            std::string("MCTS method present, but no global MCTS ") +
            "parameter file provided. Please set the mcts-params flag.");
      }

      UCTEstimator<StateQRR14, ActionQRR14>::Params uct_estimator_params;
      uct_estimator_params.gamma = params.gamma;
      uct_estimator_params.rewardBound = params.mcts_reward_bound;

      // Create the RNG required by the generative model 
      boost::mt19937 mt(2 * seed_ * (method + 1) + instance_number);
      boost::uniform_real<float> u(0.0f, 1.0f);
      URGenPtr generative_model_rng(new URGen(mt, u));

      // Create the RNG required for mcts rollouts
      boost::shared_ptr<RNG> mcts_rng(
          new RNG(3 * seed_ * (method + 1) + instance_number));

      model->initializeRNG(generative_model_rng); 
      boost::shared_ptr<ModelUpdaterSingle<StateQRR14, ActionQRR14> >
        mcts_model_updator(
            new ModelUpdaterSingle<StateQRR14, ActionQRR14>(model));
      boost::shared_ptr<IdentityStateMapping<StateQRR14> > mcts_state_mapping(
          new IdentityStateMapping<StateQRR14>);
      boost::shared_ptr<UCTEstimator<StateQRR14, ActionQRR14> > uct_estimator(
          new UCTEstimator<StateQRR14, ActionQRR14>(mcts_rng,
            uct_estimator_params));
      mcts.reset(new MCTS<StateQRR14, ActionQRR14>(uct_estimator,
            mcts_model_updator, mcts_state_mapping, mcts_params_));
    }

    boost::mt19937 mt(4 * seed_ * (method + 1) + instance_number);
    boost::uniform_real<float> u(0.0f, 1.0f);
    URGenPtr transition_rng(new URGen(mt, u));

    for (int starting_robots = 1; starting_robots <= MAX_ROBOTS;
        ++starting_robots) {

      EVALUATE_OUTPUT("Evaluating method " << method << " with " << 
          starting_robots << " robots.");
      
      StateQRR14 current_state; 
      current_state.graph_id = start_idx;
      current_state.direction = start_direction;
      current_state.num_robots_left = starting_robots;
      current_state.robot_direction = NONE;
      current_state.visible_robot = NONE;

      float reward = 0;
      float instance_distance = 0;

      EVALUATE_OUTPUT(" - start " << current_state);

      if (params.type == MCTS_TYPE) {
        mcts->restart();
        EVALUATE_OUTPUT(" - performing initial MCTS search for " +
            boost::lexical_cast<std::string>(
              params.mcts_initial_planning_time) + "s");
        for (int i = 0; i < params.mcts_initial_planning_time; ++i) {
          mcts->search(current_state); // Initial search for 10 seconds
        }
      }

      float distance_limit_pxl = 
        ((float)distance_limit_) / map.info.resolution;

      while (current_state.graph_id != goal_idx && 
          instance_distance <= distance_limit_pxl) {

        std::vector<StateQRR14> next_states;
        std::vector<float> probabilities;
        std::vector<float> rewards;

        // Deterministic system transitions
        while (true) {

          ActionQRR14 action;
          if (params.type == VI) {
            action = vi->getBestAction(current_state);
          } else if (params.type == HEURISTIC) {
            action = hs->getBestAction(current_state);
          } else if (params.type == MCTS_TYPE) {
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
          if (params.type == MCTS_TYPE) {
            EVALUATE_OUTPUT(" - performing post-action MCTS search for 1s");
            mcts->search(current_state);
          }
          EVALUATE_OUTPUT(" - auto " << current_state);
        }

        // Select next state choice based on probabilities
        int choice = select(probabilities, transition_rng);
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
          if (params.type == MCTS_TYPE) {
            // Assumes 1m/s velocity for converting distance to time
            int distance = transition_distance * map.info.resolution;
            EVALUATE_OUTPUT(" - performing post-wait MCTS search for " <<
                distance << "s");
            for (int i = 0; i < distance; ++i) {
              mcts->search(current_state);
            }
          }
        }

        EVALUATE_OUTPUT(" - manual " << current_state);
        reward += rewards[choice];
      }
      method_result.reward[starting_robots - 1] = reward;
      method_result.distance[starting_robots - 1] = 
        instance_distance * map.info.resolution;
    }
    result.results.push_back(method_result);
  }

  // Produce normalized results - distance is easy
  float normalization_distance = 
    getDistanceNormalizationValue(graph, goal_idx, start_idx) *
    map.info.resolution;

  // Normalizing rewards is more tricky - only do this if VI was one of the
  // methods, and the estimator was computed
  float normalization_reward = 1.0f;
  if (estimator) {
    normalization_reward = 
      getRewardNormalizationValue(estimator, start_idx, start_direction);
  }
  
  for (int method = 0; method < methods.size(); ++method) {
    MethodResult& method_result = result.results[method];
    MethodResult normalized_result;
    for (int i = 0; i < MAX_ROBOTS; ++i) {
      normalized_result.reward[i] = 
        method_result.reward[i] / fabs(normalization_reward);
      normalized_result.distance[i] = 
        method_result.distance[i] / normalization_distance;
    }
    result.normalized_results.push_back(normalized_result);
  }

  return result;
}

int processOptions(int argc, char** argv) {

  std::string mcts_params_file, methods_file;

  /** Define and parse the program options 
  */ 
  namespace po = boost::program_options; 
  po::options_description desc("Options"); 
  desc.add_options() 
    ("map-file", po::value<std::string>(&map_file_)->required(), 
     "YAML map file") 
    ("graph-file", po::value<std::string>(&graph_file_)->required(), 
     "YAML graph file corresponding to the map") 
    ("mcts-params", po::value<std::string>(&mcts_params_file), 
     "JSON MCTS Parameter File") 
    ("methods-file", 
     po::value<std::string>(&methods_file)->required(), 
     "JSON file containing all the different methods to be evaluated") 
    ("data-directory", po::value<std::string>(&data_directory_), 
     "Data directory (defaults to runtime directory)") 
    ("allow-robot-current", "Allow robot to be placed at current index") 
    ("allow-goal-visibility", "Allow goal visibility to affect human model")
    ("seed_", po::value<int>(&seed_), "Random seed (process number on condor)")  
    ("num-instances", po::value<int>(&num_instances_), "Number of Instances") 
    ("precompute-vi", po::value<int>(&precompute_vi_), "Precompute VI based on parameters provided in methods file. The parameters are read from the first VI instance") 
    ("visibility-range", po::value<float>(&visibility_range_), 
     "Simulator visibility range in meters.")
    ("distance-limit", po::value<float>(&distance_limit_), 
     "Max distance at which to terminate episode."); 

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
    allow_robot_current_idx_ = true;
  }
  if (vm.count("allow-goal-visibility")) {
    allow_goal_visibility_ = true;
  }

  /* Read in global MCTS parameters */
  if (!mcts_params_file.empty()) {
    Json::Value mcts_json;
    if (!readJson(mcts_params_file, mcts_json)) {
      return -1;
    }
    mcts_params_.fromJson(mcts_json);
    mcts_enabled_ = true;
  }

  /* Read in methods */
  Json::Value methods_json, methods_array;
  if (!readJson(methods_file, methods_json)) {
    return -1;
  }
  methods_array = methods_json["methods"];
  for (size_t i = 0; i < methods_array.size(); ++i) {
    Method::Params params;
    params.fromJson(methods_array[i]);
    methods_.push_back(params);
  }

  return 0;
}

int main(int argc, char** argv) {

  int ret = processOptions(argc, argv);
  if (ret != 0) {
    return ret;
  }

  std::cout << "Using random seed_: " << seed_ << std::endl;
  std::cout << "Number of instances: " << num_instances_ << std::endl;

  bwi_mapper::MapLoader mapper(map_file_);
  bwi_mapper::Graph graph;
  nav_msgs::OccupancyGrid map;
  mapper.getMap(map);
  bwi_mapper::readGraphFromFile(graph_file_, map.info, graph);

  if (precompute_vi_ != -1) {

    // Make sure correct graph id provided
    int num_vertices = boost::num_vertices(graph);
    if (precompute_vi_ < 0 || precompute_vi_ >= num_vertices) {
        throw std::runtime_error(
            std::string("The value of precompute_vi_ should be between 0 and ") 
            + boost::lexical_cast<std::string>(num_vertices));
    }

    // We are simply initializing the specified VI instance
    // Locate the first vi instance in specified methods
    BOOST_FOREACH(const Method::Params& params, methods_) {
      if (params.type == VI) {
        precomputeVI(graph, map, precompute_vi_, params);
        return 0;
      }
    }

    // If we reach here, no parameters provided in methods file
    throw std::runtime_error("No VI parameters provided in methods file");
  }

  // If we reach here, we are trying to evaluate approaches
  boost::mt19937 mt(seed_);
  boost::uniform_int<int> idx_dist(0, boost::num_vertices(graph) - 1);
  UIGen idx_gen(mt, idx_dist);
  boost::uniform_int<int> direction_dist(0, 15);
  UIGen direction_gen(mt, direction_dist);
  
  std::ofstream dfout((data_directory_ +  
        boost::lexical_cast<std::string>(seed_) + "_" +
        DISTANCE_FILE_SUFFIX).c_str());
  std::ofstream rfout((data_directory_ +  
        boost::lexical_cast<std::string>(seed_) + "_" +
        REWARD_FILE_SUFFIX).c_str());

  for (int i = 0; i < num_instances_; ++i) {
    int start_idx = idx_gen();
    int goal_idx = idx_gen();
    while (goal_idx == start_idx) {
      goal_idx = idx_gen();
    }
    int start_direction = direction_gen();
    std::cout << "#" << i << " Testing [" << start_idx << "," <<
      start_direction << "," << goal_idx << "]... " << std::endl;
    InstanceResult res = testInstance(i, graph, map, start_idx,
        start_direction, goal_idx, methods_);
    std::cout << "  ... Done" << std::endl;

    /* Output the result in CSV file */
    for (int r = 0; r < MAX_ROBOTS; ++r) {
      for (unsigned int m = 0; m < methods_.size(); ++m) {
        dfout << res.normalized_results[m].distance[r]; 
        rfout << res.normalized_results[m].reward[r]; 
        if (r != MAX_ROBOTS - 1 || m != methods_.size() - 1) {
          dfout << ",";
          rfout << ",";
        }
      }
    }

  }

  dfout.close();
  rfout.close();

  return 0;
}
