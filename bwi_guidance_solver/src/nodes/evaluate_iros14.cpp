#include<fstream>
#include<cstdlib>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <rl_pursuit/common/Util.h>
#include <rl_pursuit/planning/MCTS.h>
#include <rl_pursuit/planning/UCTEstimator.h>
#include <rl_pursuit/planning/ModelUpdaterSingle.h>
#include <rl_pursuit/planning/IdentityStateMapping.h>

#include <opencv/highgui.h>

#include <bwi_guidance_solver/heuristic_solver_iros14.h>
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
const std::string TIME_FILE_SUFFIX = "time.txt";
const std::string UTILITY_FILE_SUFFIX = "utility.txt";
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
MCTS<StateIROS14, ActionIROS14>::Params mcts_params_;
bool mcts_enabled_ = false;
bool graphical_ = false;
bool start_colocated_ = false;
bool save_images_ = false;

/* Global Data */
cv::Mat base_image_;

/* Structures used to define a single method */
const std::string METHOD_TYPE_NAMES[3] = {
  "Heuristic",
  "UCT"
};
enum MethodType {
  HEURISTIC = 0,
  MCTS_TYPE = 2,
  STATIC_BASELINE = 3 // Assumes colocation with a robot at the start of an episode
};

struct MethodResult {
  unsigned int mcts_playouts;
  unsigned int mcts_terminations;
  float reward;
  float distance;
  float time;
  float utility;
};

namespace Method {
#define PARAMS(_) \
  _(int,type,type,MCTS_TYPE) \
  _(float,gamma,gamma,1.0) \
  _(float,lambda,lambda,0.0) \
  _(bool,h_improved,h_improved,false) \
  _(float,mcts_initial_planning_time,mcts_initial_planning_time,80.0) \
  _(float,mcts_planning_time_multiplier,mcts_planning_time_multiplier,8.0) \
  _(float,mcts_reward_bound,mcts_reward_bound,250.0) \
  _(int,action_vertex_adjacency_depth,action_vertex_adjacency_depth,1) \
  _(int,max_robots_in_use,max_robots_in_use,1) \
  _(float,visibility_range,visibility_range,20.0f) \
  _(float,human_speed,human_speed,1.0f) \
  _(float,robot_speed,robot_speed,0.5f) \
  _(float,utility_multiplier,utility_multiplier,1.0f) \
  _(bool,use_shaping_reward,use_shaping_reward,true) \
  _(bool,discourage_bad_assignments,discourage_bad_assignments,false) 

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

float getOtherNormalizationValue(bwi_mapper::Graph& graph, 
    nav_msgs::OccupancyGrid& map, int goal_idx, 
    int start_idx, const Method::Params& params) {
  float distance = map.info.resolution *
    bwi_mapper::getShortestPathDistance(start_idx, goal_idx, graph);
  float best_time = distance / params.human_speed;
  return best_time;
}

float getDistanceNormalizationValue(bwi_mapper::Graph& graph, int goal_idx, 
    int start_idx) {
  return bwi_mapper::getShortestPathDistance(start_idx, goal_idx, graph);
}

/* Top level execution functions */

InstanceResult testInstance(int seed, bwi_mapper::Graph& graph, 
    nav_msgs::OccupancyGrid& map, int start_idx, int start_direction, 
    int goal_idx, const std::vector<Method::Params>& methods) {

  InstanceResult result;
  float pixel_visibility_range = visibility_range_ / map.info.resolution;

  for (size_t method = 0; method < methods.size(); ++method) {

    const Method::Params& params = methods[method];

    MethodResult method_result;

    boost::shared_ptr<HeuristicSolverIROS14> hs;
    boost::shared_ptr<MCTS<StateIROS14, ActionIROS14> > mcts;

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
      uct_estimator_params.useImportanceSampling = false;

      // Initialize the model (and random number generators)
      boost::shared_ptr<PersonModelIROS14> mcts_model(
          new PersonModelIROS14(graph, map, goal_idx, 0.0f, 
            params.max_robots_in_use, 0, params.action_vertex_adjacency_depth,
            params.visibility_range, false, params.human_speed,
            params.robot_speed, params.utility_multiplier,
            params.use_shaping_reward, params.discourage_bad_assignments));

      boost::mt19937 mt(2 * (seed + 1));
      boost::uniform_int<int> i(0, boost::num_vertices(graph) - 1);
      boost::uniform_real<float> u(0.0f, 1.0f);
      boost::poisson_distribution<int> p(1);
      URGenPtr generative_model_gen(new URGen(mt, u));
      UIGenPtr idx_gen(new UIGen(mt, i));
      PIGenPtr robot_goal_gen(new PIGen(mt, p));
      mcts_model->initializeRNG(idx_gen, generative_model_gen, robot_goal_gen);

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
    } else if (params.type == HEURISTIC) {
      hs.reset(new HeuristicSolverIROS14(map, graph, goal_idx, 
            params.h_improved, params.human_speed));
    }

    EVALUATE_OUTPUT("Evaluating method " << params);

    // Construct the evaluation model
    boost::shared_ptr<PersonModelIROS14> evaluation_model(
          new PersonModelIROS14(graph, map, goal_idx, 10.0f, 
            params.max_robots_in_use, 0, params.action_vertex_adjacency_depth,
            params.visibility_range, false, params.human_speed,
            params.robot_speed, params.utility_multiplier, 
            false, false)); // Shouldn't use shaping reward or bad assignments
    int eval_seed = 3 * (seed + 1);
    if (graphical_) {
      /* eval_seed = time(NULL); */
    }
    boost::mt19937 mt2(eval_seed);
    boost::uniform_int<int> i2(0, boost::num_vertices(graph) - 1);
    boost::uniform_real<float> u2(0.0f, 1.0f);
    boost::poisson_distribution<int> p2(1);
    URGenPtr generative_model_gen2(new URGen(mt2, u2));
    UIGenPtr idx_gen2(new UIGen(mt2, i2));
    PIGenPtr robot_goal_gen2(new PIGen(mt2, p2));
    evaluation_model->initializeRNG(idx_gen2, generative_model_gen2, robot_goal_gen2);
    boost::shared_ptr<std::vector<StateIROS14> > fv(new std::vector<StateIROS14>);
    evaluation_model->setFrameVector(fv);
    
    StateIROS14 current_state; 
    current_state.graph_id = start_idx;
    current_state.direction = start_direction;
    current_state.precision = 1.0f;
    current_state.robot_gave_direction = false;
    evaluation_model->addRobots(current_state, MAX_ROBOTS);
    evaluation_model->setState(current_state);

    if (start_colocated_) {
      float reward;
      int depth_count;
      bool terminal;
      StateIROS14 next_state;
      evaluation_model->takeAction( 
          ActionIROS14(ASSIGN_ROBOT, start_idx, DIR_UNASSIGNED), 
          reward, next_state, terminal, depth_count);
      current_state = next_state;
    }

    if (params.type == STATIC_BASELINE) {
      float robot_speed = params.robot_speed / map.info.resolution;
      float time_to_goal = 
        bwi_mapper::getShortestPathDistance(start_idx, goal_idx, graph) /
        robot_speed;
      float time_to_original_destination = 
        bwi_mapper::getShortestPathDistance(start_idx,
            current_state.robots[0].destination, graph) / robot_speed;
      float time_from_goal_to_original_destination = 
        bwi_mapper::getShortestPathDistance(goal_idx, 
            current_state.robots[0].destination, graph) / robot_speed;
      float utility_loss = 
        (time_to_goal + time_from_goal_to_original_destination - 
         time_to_original_destination);

      method_result.time = time_to_goal;
      method_result.utility = -utility_loss;
      method_result.reward = 
        -time_to_goal - params.utility_multiplier * utility_loss;
      method_result.distance = time_to_goal * params.robot_speed;
      result.results.push_back(method_result);

      continue; // Continue to the next method directly, no evaluation required
    }

    float instance_reward = 0.0f;
    float instance_distance = 0.0f;
    float instance_time = 0.0f;
    float instance_utility = 0.0f;

    EVALUATE_OUTPUT(" - Start " << current_state);
    if (graphical_) {
      cv::Mat out_img = base_image_.clone();
      evaluation_model->drawState(current_state, out_img);
      cv::imshow("out", out_img);
      //cv::waitKey(100);
      if (params.type == HEURISTIC) {
        // Introduce a 10 second delay so that the observer can get oriented
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
      }
    }

    method_result.mcts_terminations = 0;
    method_result.mcts_playouts = 0;
    if (params.type == MCTS_TYPE) {
      mcts->restart();
      EVALUATE_OUTPUT(" - Performing initial MCTS search for " +
          boost::lexical_cast<std::string>(
            params.mcts_initial_planning_time) + "s");
      for (int i = 0; i < 10 * params.mcts_initial_planning_time; ++i) {
        unsigned int playouts, terminations;
        playouts = mcts->search(current_state, terminations);
        method_result.mcts_playouts += playouts;
        method_result.mcts_terminations += terminations;
      }
    }

    EVALUATE_OUTPUT("     Found " << method_result.mcts_terminations << 
        " terminations in " << method_result.mcts_playouts << " playouts");

    float distance_limit_pxl = 
      ((float)distance_limit_) / map.info.resolution;

    while (instance_distance <= distance_limit_pxl) {

      std::vector<ActionIROS14> actions;
      evaluation_model->getActionsAtState(current_state, actions);
      ActionIROS14 action;
      if (params.type == MCTS_TYPE) {
        action = mcts->selectWorldAction(current_state);
      } else if (params.type == HEURISTIC) {
        action = hs->getBestAction(current_state, evaluation_model);
      }
      // std::cout << "Select: " << std::endl;
      // int choice;
      // std::cin >> choice;
      // action = actions[choice];
      EVALUATE_OUTPUT(" - Method selects: " << action);
      float reward;
      StateIROS14 next_state;
      bool terminal;
      int depth_count;
      float time_loss, utility_loss;
      evaluation_model->takeAction(action, reward, next_state, terminal,
          depth_count);
      evaluation_model->getLossesInPreviousTransition(time_loss, utility_loss);
      float transition_distance = 
        bwi_mapper::getEuclideanDistance(next_state.graph_id,
            current_state.graph_id, graph);
      instance_distance += transition_distance;
      instance_reward += reward;
      instance_time += time_loss;
      instance_utility -= utility_loss;
      current_state = next_state;
      EVALUATE_OUTPUT(" - Next state: " << current_state);
      EVALUATE_OUTPUT("     Reward: " << reward << 
          ", Distance: " << transition_distance * map.info.resolution <<
          ", Time Lost: " << time_loss <<
          ", Utility Lost: " << utility_loss <<
          ", Depth Count: " << depth_count);

      if (action.type == WAIT) {
        // Prune old visits before searching
        if (params.type == MCTS_TYPE) {
          EVALUATE_OUTPUT(" - Cleared existing MCTS search tree");
          mcts->restart();
        }

        float total_time = 0.0f;
        BOOST_FOREACH(StateIROS14& state, *fv) {
          if (graphical_) {
            cv::Mat out_img = base_image_.clone();
            evaluation_model->drawState(state, out_img);
            cv::imshow("out", out_img);
            //cv::waitKey(50);
          }
          if (params.type == MCTS_TYPE) {
            for (int i = 0; i < params.mcts_planning_time_multiplier; ++i) {
              total_time += 0.1f;
              unsigned int terminations;
              mcts->search(current_state, terminations);
            }
          } else {
            if (graphical_) {
              // Sleep this thread manually
              boost::this_thread::sleep(boost::posix_time::milliseconds(10));
            }
          }
        }
        if (params.type == MCTS_TYPE) {
          EVALUATE_OUTPUT(" - Performed MCTS search for " << total_time << "s");
        }
      }

      if (terminal) {
        break;
      }

    }

    // Remove the shaping reward from the result tally if it was used.
    // The evaluation model should not use this shaping reward in the first place
    // if (params.use_shaping_reward) {
    //   float distance = map.info.resolution *
    //     bwi_mapper::getShortestPathDistance(start_idx, goal_idx, graph);
    //   float time = distance / params.human_speed;
    //   instance_reward -= time;
    // }

    method_result.reward = instance_reward;
    method_result.time = instance_time;
    method_result.utility = instance_utility;
    method_result.distance = instance_distance * map.info.resolution;
    result.results.push_back(method_result);
  }

  // Produce normalized results - distance is easy
  float normalization_distance = 
    getDistanceNormalizationValue(graph, goal_idx, start_idx) *
    map.info.resolution;

  for (int method = 0; method < methods.size(); ++method) {
    float normalization_other = getOtherNormalizationValue(graph, map,
        goal_idx, start_idx, methods[method]);
    
    MethodResult& method_result = result.results[method];
    MethodResult normalized_result;
    normalized_result.mcts_playouts = method_result.mcts_playouts;
    normalized_result.mcts_terminations = method_result.mcts_terminations;
    normalized_result.time = 
      method_result.time / fabs(normalization_other);
    normalized_result.utility = 
      method_result.utility / fabs(normalization_other);
    normalized_result.reward = 
      method_result.reward / fabs(normalization_other);
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
    ("graphical", "Use graphical interface for debugging purposes")
    ("start-colocated", "Start state coincides with a robot home base")
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

  if (vm.count("graphical")) {
    graphical_ = true;
  }
  if (vm.count("start-colocated")) {
    start_colocated_ = true;
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

  if (graphical_) {
    cv::namedWindow("out");
    cvStartWindowThread();
  }

  std::cout << "Using random seed: " << seed_ << std::endl;
  std::cout << "Number of instances: " << num_instances_ << std::endl;
  std::cout << "Map File: " << map_file_ << std::endl;
  std::cout << "Graph File: " << graph_file_ << std::endl;
  std::cout << "Visibility Range: " << visibility_range_ << std::endl;
  std::cout << "Distance Limit: " << distance_limit_ << std::endl;

  bwi_mapper::MapLoader mapper(map_file_);
  bwi_mapper::Graph graph;
  nav_msgs::OccupancyGrid map;
  mapper.getMap(map);
  bwi_mapper::readGraphFromFile(graph_file_, map.info, graph);
  mapper.drawMap(base_image_);

  // If we reach here, we are trying to evaluate approaches
  std::ofstream dfout((data_directory_ +  
        boost::lexical_cast<std::string>(seed_) + "_" +
        DISTANCE_FILE_SUFFIX).c_str());
  std::ofstream rfout((data_directory_ +  
        boost::lexical_cast<std::string>(seed_) + "_" +
        REWARD_FILE_SUFFIX).c_str());
  std::ofstream timefout((data_directory_ +  
        boost::lexical_cast<std::string>(seed_) + "_" +
        TIME_FILE_SUFFIX).c_str());
  std::ofstream ufout((data_directory_ +  
        boost::lexical_cast<std::string>(seed_) + "_" +
        UTILITY_FILE_SUFFIX).c_str());
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
    if (start_colocated_) {
      start_idx = start_idx % 10;
      start_idx = ROBOT_HOME_BASE[start_idx];
    }
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
      timefout << res.normalized_results[m].time; 
      ufout << res.normalized_results[m].utility; 
      pfout << res.results[m].mcts_playouts; 
      tfout << res.results[m].mcts_terminations; 
      if (m != methods_.size() - 1) {
        dfout << ",";
        rfout << ",";
        timefout << ",";
        ufout << ",";
        pfout << ",";
        tfout << ",";
      }
    }

    dfout << std::endl;
    rfout << std::endl;
    timefout << std::endl;
    ufout << std::endl;
    pfout << std::endl;
    tfout << std::endl;

  }

  dfout.close();
  rfout.close();
  timefout.close();
  ufout.close();
  pfout.close();
  tfout.close();

  return 0;
}
