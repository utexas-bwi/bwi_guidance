#include <boost/thread/thread.hpp>
#include <bwi_guidance_solver/mrn/abstract_mapping.h>
#include <bwi_guidance_solver/mrn/restricted_model.h>
#include <bwi_guidance_solver/mrn/person_model.h>
#include <bwi_guidance_solver/mrn/mcts_solver.h>
#include <bwi_guidance_solver/mrn/single_robot_solver.h>
#include <bwi_mapper/graph.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_rl/planning/ModelUpdaterSingle.h>
#include <bwi_tools/resource_resolver.h>
#include <opencv/highgui.h>

using namespace bwi_guidance_solver;
using namespace bwi_guidance_solver::mrn;

void constructFinalImage(cv::Mat &img_final, 
                         const std::string &str_sr,
                         const cv::Mat &img_sr,
                         const std::string &str_mcts,
                         const cv::Mat &img_mcts) {
  
  int text_height = 70;
  int crop_border = 30;
  int padding = 15;
  float scale = 1.5f;

  int final_img_height = img_sr.rows + 2 * text_height - 2 * crop_border;
  int final_img_width = 2 * img_sr.cols - 4 * crop_border;

  img_final = cv::Mat(final_img_height, final_img_width, CV_8UC3, cv::Scalar(255, 255, 255));

  cv::Rect source_roi(cv::Point(crop_border, crop_border), cv::Size(img_sr.cols - 2 * crop_border, img_sr.rows - 2 * crop_border));

  // Copy over left image.
  cv::Rect dest_roi(cv::Point(0, text_height), cv::Size(img_sr.cols - 2 * crop_border, img_sr.rows - 2 * crop_border));
  cv::Mat destination_img = img_final(dest_roi);
  cv::Mat cropped_sr = img_sr(source_roi);
  cropped_sr.copyTo(destination_img);
  
  // Copy over right image.
  dest_roi = cv::Rect(cv::Point(final_img_width/2, text_height), cv::Size(img_sr.cols - 2 * crop_border, img_sr.rows - 2 * crop_border));
  destination_img = img_final(dest_roi);
  cv::Mat cropped_mcts = img_mcts(source_roi);
  cropped_mcts.copyTo(destination_img);

  // Left side text.
  cv::putText(img_final, "Starting-Robot Solution", cv::Point2f(padding, text_height - padding), CV_FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 3);
  cv::putText(img_final, str_sr, cv::Point2f(padding, final_img_height - 2 * padding), CV_FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 3);

  // Right side text.
  cv::putText(img_final, "Multi-Robot MCTS Solution", cv::Point2f(final_img_width / 2 + padding, text_height - padding), CV_FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 3);
  cv::putText(img_final, str_mcts, cv::Point2f(final_img_width / 2 + padding, final_img_height - 2 * padding), CV_FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 3);
}

int main(int argc, const char *argv[]) {

  int seed = 0;
  if (argc > 1) {
    seed = atoi(argv[1]);
  }

  std::string map_file = "package://bwi_guidance/maps/map3.yaml";
  std::string graph_file = "package://bwi_guidance/maps/graph_map3.yaml";
  std::string robot_home_base_file = "package://bwi_guidance_solver/experiments/map3_robot_base.txt";

  // Initialize map, graph and model.
  cv::Mat base_image;
  nav_msgs::OccupancyGrid map;
  bwi_mapper::Graph graph;

  map_file = bwi_tools::resolveRosResource(map_file);
  graph_file = bwi_tools::resolveRosResource(graph_file);
  robot_home_base_file = bwi_tools::resolveRosResource(robot_home_base_file);
  bwi_mapper::MapLoader mapper(map_file);
  mapper.getMap(map);
  mapper.drawMap(base_image);
  bwi_mapper::readGraphFromFile(graph_file, map.info, graph);
  std::cout << "map, graph read" << std::endl;

  std::vector<int> robot_home_base;
  readRobotHomeBase(robot_home_base_file, robot_home_base);

  std::cout << "map, graph and home base data read" << std::endl;

  MotionModel::Ptr motion_model(new MotionModel(graph,
                                                0.5f / map.info.resolution,
                                                1.0f / map.info.resolution));
  TaskGenerationModel::Ptr task_generation_model(new TaskGenerationModel(robot_home_base, graph, 1.0f));

  HumanDecisionModel::Ptr human_decision_model(new HumanDecisionModel(graph));

  /* Enable visualization. */
  cv::namedWindow("out");
  cvStartWindowThread();

  Domain::Params domain_params;

  float unused_reward, unused_time_loss, unused_utility_loss;
  bool unused_terminal;
  int unused_depth_count;
  std::vector<State> frame_vector;

  boost::shared_ptr<RNG> rng(new RNG(seed));
  int num_vertices = boost::num_vertices(graph);
  int start_robot_idx = rng->randomInt(robot_home_base.size() - 1);
  int start_idx = (domain_params.start_colocated) ? robot_home_base[start_robot_idx] : rng->randomInt(num_vertices - 1);
  int goal_idx = rng->randomInt(num_vertices - 1);
  while (goal_idx == start_idx) {
    // Measure distance between goal_idx and start_idx.
    goal_idx = rng->randomInt(num_vertices - 1);
  }

  std::cout << "Instance params initialized..." << std::endl;

  RestrictedModel::Params params;
  params.frame_rate = 10.0f;
  boost::shared_ptr<RestrictedModel> model;
  model.reset(new RestrictedModel(graph, map, goal_idx, motion_model, human_decision_model, task_generation_model, params));

  std::cout << "Model initialized..." << std::endl;

  // Setup both the solvers
  boost::shared_ptr<SingleRobotSolver> sr;
  sr.reset(new SingleRobotSolver);
  std::string base_directory = "/tmp";
  Json::Value empty_json; // Force the single robot solver to use default parameters.
  sr->initialize(domain_params, empty_json, map, graph, robot_home_base, base_directory);
  sr->reset(0, goal_idx);

  std::cout << "Single robot solver initialized..." << std::endl;

  boost::shared_ptr<ModelUpdaterSingle<ExtendedState, Action> >
    mcts_model_updator(new ModelUpdaterSingle<ExtendedState, Action>(model));
  boost::shared_ptr<StateMapping<ExtendedState> > mcts_state_mapping;
  mcts_state_mapping.reset(new AbstractMapping);

  MultiThreadedMCTS<ExtendedState, ExtendedStateHash, Action>::Params mcts_params;
  boost::shared_ptr<MultiThreadedMCTS<ExtendedState, ExtendedStateHash, Action> > mcts;
  mcts.reset(new MultiThreadedMCTS<ExtendedState, ExtendedStateHash, Action>(sr,
                                                                             mcts_model_updator,
                                                                             mcts_state_mapping,
                                                                             rng,
                                                                             mcts_params));

  std::cout << "MCTS solver initialized..." << std::endl;

  // Original state - state 1.
  ExtendedState current_state_sr, next_state;
  current_state_sr.loc_node = start_idx;
  current_state_sr.loc_p = 0.0f;
  current_state_sr.loc_prev = start_idx;
  current_state_sr.assist_type = NONE;
  current_state_sr.assist_loc = NONE;
  current_state_sr.robots.resize(robot_home_base.size());
  int robot_counter = 0;
  BOOST_FOREACH(RobotState &robot, current_state_sr.robots) {
    robot.loc_u = robot_home_base[robot_counter];
    robot.loc_p = 0.0f;
    robot.loc_v = robot.loc_u;
    robot.help_destination = NONE;
    task_generation_model->generateNewTaskForRobot(robot_counter, robot, *rng);
    ++robot_counter;
  }
  std::cout << "Original state: " << current_state_sr << std::endl;

  model->takeAction(current_state_sr, Action(ASSIGN_ROBOT, start_robot_idx, start_idx),
                            unused_reward, next_state, unused_terminal, unused_depth_count,
                            rng);
  current_state_sr = next_state;
  current_state_sr.prev_action = Action(WAIT);
  current_state_sr.released_locations.clear();

  ExtendedState current_state_mcts = current_state_sr;
  unsigned int unused_rollout_termination_count;
  std::cout << "Starting initial mcts search" << std::endl;
  mcts->search(current_state_mcts, unused_rollout_termination_count, 80.0f);
  std::cout << "Ending initial mcts search" << std::endl;

  // Draw the original state and wait 8 seconds.
  cv::Mat img_sr = base_image.clone(), img_mcts;
  model->drawState(current_state_sr, img_sr);
  img_mcts = img_sr.clone();
  cv::Mat img_final;
  std::string text_sr = "Episode Starting...";
  constructFinalImage(img_final, "Episode Starting...", img_sr, "Episode Starting...", img_mcts);
  cv::imshow("out", img_final);
  boost::this_thread::sleep(boost::posix_time::milliseconds(5000.0f));

  std::cout << "Initiailizing video write." << std::endl;
  cv::VideoWriter writer("/tmp/guidance.avi", CV_FOURCC('I', 'Y', 'U', 'V'), params.frame_rate, img_final.size());
  for (int i = 0; i < 5.0f / params.frame_rate; ++i) {
    writer.write(img_final);
  }
  std::cout << "Video write initialization complete." << std::endl;

  std::vector<cv::Mat> img_arr_sr, img_arr_mcts;
  std::vector<std::string> str_arr_sr, str_arr_mcts;

  while (current_state_sr.loc_node != goal_idx) {
    Action action = sr->getBestAction(current_state_sr);
    model->takeAction(current_state_sr, action, unused_reward, next_state, unused_terminal,
                              unused_depth_count, rng, unused_time_loss, unused_utility_loss, frame_vector);

    BOOST_FOREACH(State &state, frame_vector) {
      img_sr = base_image.clone();
      model->drawState(state, img_sr);
      std::stringstream ss;
      ss << action;
      img_arr_sr.push_back(img_sr);
      str_arr_sr.push_back(ss.str());
    }

    current_state_sr = next_state;
  }

  img_arr_sr.push_back(img_sr);
  str_arr_sr.push_back("Episode Finished");

  while (current_state_mcts.loc_node != goal_idx) {
    Action action = mcts->selectWorldAction(current_state_mcts);
    model->takeAction(current_state_mcts, action, unused_reward, next_state, unused_terminal,
                              unused_depth_count, rng, unused_time_loss, unused_utility_loss, frame_vector);

    BOOST_FOREACH(State &state, frame_vector) {
      img_mcts = base_image.clone();
      model->drawState(state, img_mcts);
      std::stringstream ss;
      ss << action;
      img_arr_mcts.push_back(img_mcts);
      str_arr_mcts.push_back(ss.str());
    }

    current_state_mcts = next_state;

    if (action.type == WAIT) {
      mcts->restart();
      mcts->search(current_state_mcts, unused_rollout_termination_count, 100.0f);
    }

  }

  img_arr_mcts.push_back(img_mcts);
  str_arr_mcts.push_back("Episode Finished");

  int max_counter = std::max(img_arr_sr.size(), img_arr_mcts.size());

  for (int i = 0; i < max_counter; ++i) {
    int idx_sr = (i >= img_arr_sr.size()) ? img_arr_sr.size() - 1 : i;
    int idx_mcts = (i >= img_arr_mcts.size()) ? img_arr_mcts.size() - 1 : i;
    constructFinalImage(img_final, str_arr_sr[idx_sr], img_arr_sr[idx_sr], str_arr_mcts[idx_mcts], img_arr_mcts[idx_mcts]);
    writer.write(img_final);
  }

  return 0;
}
