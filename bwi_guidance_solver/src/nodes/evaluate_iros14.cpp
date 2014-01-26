#include<fstream>
#include<cstdlib>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <rl_pursuit/common/Util.h>
#include <rl_pursuit/planning/MCTS.h>
#include <rl_pursuit/planning/UCTEstimator.h>
#include <rl_pursuit/planning/ModelUpdaterSingle.h>
#include <rl_pursuit/planning/IdentityStateMapping.h>

#include <bwi_guidance_solver/person_model_iros14.h>
#include <bwi_guidance_solver/utils.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_mapper/map_utils.h>

#define MAX_ROBOTS 10

#ifdef EVALUATE_DEBUG
#define EVALUATE_OUTPUT(x) std::cout << x << std::endl
#else
#define EVALUATE_OUTPUT(x) ((void) 0)
#endif

using namespace bwi_guidance;

/* Constants */
const std::string MODEL_FILE_SUFFIX = "model";
const std::string DISTANCE_FILE_SUFFIX = "distance.txt";
const std::string REWARD_FILE_SUFFIX = "reward.txt";
const std::string PLAYOUTS_FILE_SUFFIX = "playouts.txt";
const std::string TERMINATIONS_FILE_SUFFIX = "terminations.txt";

/* Parameters (with their defaults) */
std::string data_directory_ = "";
std::string map_file_ = "";
std::string graph_file_ = "";
int seed_ = 0;
int num_instances_ = 1;
float distance_limit_ = 300.f;
float visibility_range_ = 0.0f; // Infinite visibility
bool allow_goal_visibility_ = false;
MCTS<StateIROS14, ActionIROS14>::Params mcts_params_;
bool mcts_enabled_ = false;

/* Structures used to define a single method */
const std::string METHOD_TYPE_NAMES[3] = {
  "Heuristic",
  "UCT"
};
enum MethodType {
  HEURISTIC = 0,
  MCTS_TYPE = 1
};

struct MethodResult {
  unsigned int mcts_playouts;
  unsigned int mcts_terminations;
  float reward;
  float distance;
};

namespace Method {
#define PARAMS(_) \
  _(int,type,type,MCTS_TYPE) \
  _(float,gamma,gamma,1.0) \
  _(float,lambda,lambda,0.0) \
  _(float,mcts_initial_planning_time,mcts_initial_planning_time,10.0) \
  _(float,mcts_planning_time_multiplier,mcts_planning_time_multiplier,1.0) \
  _(float,mcts_reward_bound,mcts_reward_bound,5000.0) \
  _(bool,mcts_importance_sampling,mcts_importance_sampling,false) 

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

float getDistanceNormalizationValue(bwi_mapper::Graph& graph, int goal_idx, 
    int start_idx) {
  std::vector<size_t> temp_path;
  return bwi_mapper::getShortestPathWithDistance(start_idx, goal_idx, temp_path,
      graph);
}

/* Top level execution functions */

InstanceResult testInstance(int seed, bwi_mapper::Graph& graph, 
    nav_msgs::OccupancyGrid& map, int start_idx, int start_direction, 
    int goal_idx, const std::vector<Method::Params>& methods) {

  InstanceResult result;
  float pixel_visibility_range = visibility_range_ / map.info.resolution;

  for (size_t method = 0; method < methods.size(); ++method) {

    const Method::Params& params = methods[method];

    // Initialize the model (and random number generators)
    boost::shared_ptr<PersonModelIROS14> mcts_model(
        new PersonModelIROS14(graph, map, goal_idx));

    boost::mt19937 mt(2 * (seed + 1));
    boost::uniform_int<int> i(0, boost::num_vertices(graph) - 1);
    boost::uniform_real<float> u(0.0f, 1.0f);
    boost::poisson_distribution<int> p(2);
    URGenPtr generative_model_gen(new URGen(mt, u));
    UIGenPtr idx_gen(new UIGen(mt, i));
    PIGenPtr robot_goal_gen(new PIGen(mt, p));
    mcts_model->initializeRNG(idx_gen, generative_model_gen, robot_goal_gen);

    boost::shared_ptr<PersonModelIROS14> evaluation_model(
        new PersonModelIROS14(graph, map, goal_idx));
    boost::mt19937 mt2(3 * (seed + 1));
    boost::uniform_int<int> i2(0, boost::num_vertices(graph) - 1);
    boost::uniform_real<float> u2(0.0f, 1.0f);
    boost::poisson_distribution<int> p2(2);
    URGenPtr generative_model_gen2(new URGen(mt, u2));
    UIGenPtr idx_gen2(new UIGen(mt, i2));
    PIGenPtr robot_goal_gen2(new PIGen(mt, p2));
    evaluation_model->initializeRNG(idx_gen2, generative_model_gen2, robot_goal_gen2);

    MethodResult method_result;

    boost::shared_ptr<MCTS<StateIROS14, ActionIROS14> > mcts;

    // Until we get a working copy of the heuristic
    assert(params.type == MCTS_TYPE);

    if (params.type == MCTS_TYPE) {

      if (!mcts_enabled_) {
        throw std::runtime_error(
            std::string("MCTS method present, but no global MCTS ") +
            "parameter file provided. Please set the mcts-params flag.");
      }

      UCTEstimator<StateIROS14, ActionIROS14>::Params uct_estimator_params;
      uct_estimator_params.gamma = params.gamma;
      uct_estimator_params.lambda = params.lambda;
      uct_estimator_params.rewardBound = params.mcts_reward_bound;
      uct_estimator_params.useImportanceSampling =
        params.mcts_importance_sampling;

      // Create the RNG required for mcts rollouts
      boost::shared_ptr<RNG> mcts_rng(new RNG(3 * (seed + 1)));

      boost::shared_ptr<ModelUpdaterSingle<StateIROS14, ActionIROS14> >
        mcts_model_updator(
            new ModelUpdaterSingle<StateIROS14, ActionIROS14>(mcts_model));
      boost::shared_ptr<IdentityStateMapping<StateIROS14> > mcts_state_mapping(
          new IdentityStateMapping<StateIROS14>);
      boost::shared_ptr<UCTEstimator<StateIROS14, ActionIROS14> > uct_estimator(
          new UCTEstimator<StateIROS14, ActionIROS14>(mcts_rng,
            uct_estimator_params));
      mcts.reset(new MCTS<StateIROS14, ActionIROS14>(uct_estimator,
            mcts_model_updator, mcts_state_mapping, mcts_params_));
    }

    EVALUATE_OUTPUT("Evaluating method " << params);
    
    StateIROS14 current_state; 
    current_state.graph_id = start_idx;
    current_state.direction = start_direction;
    evaluation_model->addRobots(current_state, MAX_ROBOTS);
    evaluation_model->setState(current_state);

    float instance_reward = 0;
    float instance_distance = 0;

    EVALUATE_OUTPUT(" - start " << current_state);

    method_result.mcts_terminations = 0;
    method_result.mcts_playouts = 0;
    if (params.type == MCTS_TYPE) {
      mcts->restart();
      EVALUATE_OUTPUT(" - performing initial MCTS search for " +
          boost::lexical_cast<std::string>(
            params.mcts_initial_planning_time) + "s");
      for (int i = 0; i < params.mcts_initial_planning_time; ++i) {
        unsigned int playouts, terminations;
        playouts = mcts->search(current_state, terminations);
        method_result.mcts_playouts = playouts;
        method_result.mcts_terminations += terminations;
      }
      EVALUATE_OUTPUT("    Done...");
    }

    float distance_limit_pxl = 
      ((float)distance_limit_) / map.info.resolution;

    while (instance_distance <= distance_limit_pxl) {

      ActionIROS14 action;
      action = mcts->selectWorldAction(current_state);
      EVALUATE_OUTPUT("   action: " << action);
      float reward;
      StateIROS14 next_state;
      bool terminal;
      evaluation_model->takeAction(action, reward, next_state, terminal);
      float transition_distance = 
        bwi_mapper::getEuclideanDistance(next_state.graph_id,
            current_state.graph_id, graph);
      instance_distance += transition_distance;
      instance_reward += reward;
      current_state = next_state;
      EVALUATE_OUTPUT(" - ns " << current_state);

      if (terminal) {
        break;
      }

      if (params.type == MCTS_TYPE) {
        // Assumes 1m/s velocity for converting distance to time
        // TODO account for human speed from method params
        int distance = transition_distance * map.info.resolution;
        distance += params.mcts_planning_time_multiplier;
        EVALUATE_OUTPUT(" - performing post-wait MCTS search for " <<
            distance << "s");
        for (int i = 0; i < distance; ++i) {
          unsigned int terminations;
          mcts->search(current_state, terminations);
        }
      }
    }

    method_result.reward = instance_reward;
    method_result.distance = instance_distance * map.info.resolution;
    result.results.push_back(method_result);
  }

  // Produce normalized results - distance is easy
  float normalization_distance = 
    getDistanceNormalizationValue(graph, goal_idx, start_idx) *
    map.info.resolution;

  // TODO figure out a good way of normalizing the total reward
  float normalization_reward = 1.0f;
  
  for (int method = 0; method < methods.size(); ++method) {
    MethodResult& method_result = result.results[method];
    MethodResult normalized_result;
    normalized_result.mcts_playouts = method_result.mcts_playouts;
    normalized_result.mcts_terminations = method_result.mcts_terminations;
    normalized_result.reward = 
      method_result.reward / fabs(normalization_reward);
    normalized_result.distance = 
      method_result.distance / normalization_distance;
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
    ("allow-goal-visibility", "Allow goal visibility to affect human model")
    ("seed_", po::value<int>(&seed_), "Random seed (process number on condor)")  
    ("num-instances", po::value<int>(&num_instances_), "Number of Instances") 
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
  std::cout << "Methods File: " << methods_file << std::endl;
  Json::Value methods_json, methods_array;
  if (!readJson(methods_file, methods_json)) {
    return -1;
  }
  methods_array = methods_json["methods"];
  std::cout << "Processing Methods: " << std::endl;
  for (size_t i = 0; i < methods_array.size(); ++i) {
    Method::Params params;
    params.fromJson(methods_array[i]);
    methods_.push_back(params);
    std::cout << "Method " << i << ": " << params << std::endl;
  }

  if (num_instances_ < 1) {
    std::cerr << "ERROR: num-instances must be positive!!" << std::endl; 
    return -1;
  }

  return 0;
}

int main(int argc, char** argv) {

  int ret = processOptions(argc, argv);
  if (ret != 0) {
    return ret;
  }

  std::cout << "Using random seed: " << seed_ << std::endl;
  std::cout << "Number of instances: " << num_instances_ << std::endl;
  std::cout << "Map File: " << map_file_ << std::endl;
  std::cout << "Graph File: " << graph_file_ << std::endl;
  std::cout << "Visibility Range: " << visibility_range_ << std::endl;
  std::cout << "Distance Limit: " << distance_limit_ << std::endl;
  std::cout << "Allow Goal Visibiliy: " << allow_goal_visibility_ << std::endl;

  bwi_mapper::MapLoader mapper(map_file_);
  bwi_mapper::Graph graph;
  nav_msgs::OccupancyGrid map;
  mapper.getMap(map);
  bwi_mapper::readGraphFromFile(graph_file_, map.info, graph);

  // If we reach here, we are trying to evaluate approaches
  std::ofstream dfout((data_directory_ +  
        boost::lexical_cast<std::string>(seed_) + "_" +
        DISTANCE_FILE_SUFFIX).c_str());
  std::ofstream rfout((data_directory_ +  
        boost::lexical_cast<std::string>(seed_) + "_" +
        REWARD_FILE_SUFFIX).c_str());
  std::ofstream pfout((data_directory_ +  
        boost::lexical_cast<std::string>(seed_) + "_" +
        PLAYOUTS_FILE_SUFFIX).c_str());
  std::ofstream tfout((data_directory_ +  
        boost::lexical_cast<std::string>(seed_) + "_" +
        TERMINATIONS_FILE_SUFFIX).c_str());

  for (int i = 0; i < num_instances_; ++i) {

    boost::mt19937 mt(seed_ + i);
    boost::uniform_int<int> idx_dist(0, boost::num_vertices(graph) - 1);
    UIGen idx_gen(mt, idx_dist);
    boost::uniform_int<int> direction_dist(0, 15);
    UIGen direction_gen(mt, direction_dist);
    
    int start_idx = idx_gen();
    int goal_idx = idx_gen();
    while (goal_idx == start_idx) {
      goal_idx = idx_gen();
    }
    int start_direction = direction_gen();
    std::cout << "#" << i << " Testing [" << start_idx << "," <<
      start_direction << "," << goal_idx << "]... " << std::endl;
    std::cout << "Using seed: " << seed_ + i << ", " << seed_ << "+" << i << std::endl; 
    InstanceResult res = testInstance(seed_ + i, graph, map, start_idx,
        start_direction, goal_idx, methods_);
    std::cout << "  ... Done" << std::endl;

    /* Output the result in CSV file */
    for (unsigned int m = 0; m < methods_.size(); ++m) {
      dfout << res.normalized_results[m].distance; 
      rfout << res.normalized_results[m].reward; 
      pfout << res.results[m].mcts_playouts; 
      tfout << res.results[m].mcts_terminations; 
      if (m != methods_.size() - 1) {
        dfout << ",";
        rfout << ",";
        pfout << ",";
        tfout << ",";
      }
    }

    dfout << std::endl;
    rfout << std::endl;
    pfout << std::endl;
    tfout << std::endl;

  }

  dfout.close();
  rfout.close();
  pfout.close();
  tfout.close();

  return 0;
}
