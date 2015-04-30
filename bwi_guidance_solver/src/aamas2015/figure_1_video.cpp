#include <boost/thread/thread.hpp>
#include <bwi_guidance_solver/mrn/restricted_model.h>
#include <bwi_guidance_solver/mrn/person_model.h>
#include <bwi_mapper/graph.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_tools/resource_resolver.h>
#include <opencv/highgui.h>

using namespace bwi_guidance_solver;
using namespace bwi_guidance_solver::mrn;

int main(int argc, const char *argv[]) {

  int seed = 0;
  if (argc > 1) {
    seed = atoi(argv[1]);
  }

  std::string map_file = "package://bwi_guidance/maps/map3.yaml";
  std::string graph_file = "package://bwi_guidance/maps/graph_map3.yaml";
  std::string robot_home_base_file = "package://bwi_guidance/experiments/map3_robot_base.txt";

  // Initialize map, graph and model.
  cv::Mat base_image;
  nav_msgs::OccupancyGrid map;
  bwi_mapper::Graph graph;

  map_file = bwi_tools::resolveRosResource(map_file);
  graph_file = bwi_tools::resolveRosResource(graph_file);
  bwi_mapper::MapLoader mapper(map_file);
  mapper.getMap(map);
  mapper.drawMap(base_image);
  bwi_mapper::readGraphFromFile(graph_file, map.info, graph);
  std::vector<int> robot_home_base;
  readRobotHomeBase(robot_home_base_file, robot_home_base);

  MotionModel::Ptr motion_model(new MotionModel(graph,
                                                0.5f / map.info.resolution,
                                                1.0f / map.info.resolution));
  TaskGenerationModel::Ptr task_generation_model(new TaskGenerationModel(robot_home_base, graph, 1.0f));

  HumanDecisionModel::Ptr human_decision_model(new HumanDecisionModel(graph));

  /* Enable visualization. */
  PersonModel::Params params;
  params.frame_rate = 10.0f;
  cv::namedWindow("out");
  cvStartWindowThread();

  PersonModel model(graph, map, 12, motion_model, human_decision_model, task_generation_model, params);

  /* Also create an extended model with defaults. */
  RestrictedModel::Params extended_params;
  extended_params.frame_rate = 10.0f;
  extended_params.max_assigned_robots = 2;
  RestrictedModel extended_model(graph, map, 12, motion_model, human_decision_model, task_generation_model, extended_params);

  float unused_reward, unused_time_loss, unused_utility_loss;
  bool unused_terminal;
  int unused_depth_count;
  std::vector<State> frame_vector;

  boost::shared_ptr<RNG> rng(new RNG(seed));
  int num_vertices = boost::num_vertices(graph_);
  int start_robot_idx = rng.randomInt(robot_home_base.size() - 1);
  int start_idx = (params_.start_colocated) ? robot_home_base[start_robot_idx] : rng.randomInt(num_vertices - 1);
  int goal_idx = rng.randomInt(num_vertices - 1);
  while (true) {
    // Measure distance between goal_idx and start_idx.
    goal_idx = rng.randomInt(num_vertices - 1);
  }

  // Original state - state 1.
  ExtendedState current_state_sr, next_state;
  current_state_sr.loc_node = start_idx;
  current_state_sr.loc_p = 0.0f;
  current_state_sr.loc_prev = start_idx;
  current_state_sr.assist_type = NONE;
  current_state_sr.assist_loc = NONE;
  current_state_sr.robots.resize(robot_home_base_.size());
  int robot_counter = 0;
  BOOST_FOREACH(RobotState &robot, current_state_sr.robots) {
    robot.loc_u = robot_home_base_[robot_counter];
    robot.loc_p = 0.0f;
    robot.loc_v = robot.loc_u;
    robot.help_destination = NONE;
    task_generation_model->generateNewTaskForRobot(robot_counter, robot, *evaluation_rng);
    ++robot_counter;
  }
  std::cout << "Original state: " << current_state_sr << std::endl;

  extended_model.takeAction(current_state_sr, Action(ASSIGN_ROBOT, start_robot_idx, start_idx),
                            unused_reward, next_state, unused_terminal, unused_depth_count,
                            evaluation_rng);
  current_state_sr = next_state;
  current_state_sr.prev_action = Action(WAIT);
  current_state_sr.released_locations.clear();

  ExtendedState current_state_mcts = current_state_sr;

  // Draw the original state and wait 8 seconds.
  cv::Mat img_sr = base_image.clone(), img_mcts;
  model.drawState(current_state_sr, img_sr);
  img_mcts = img_sr.clone();
  cv::Mat img_final;
  std::string text_sr = "Episode Starting...";
  constructFinalImage(img_final, "Episode Starting...", img_sr, "Episode Starting...", Startimg_mcts);
  cv::imshow("out", img_final);
  boost::this_thread::sleep(boost::posix_time::milliseconds(5000.0f));

  cv::VideoWriter writer("/tmp/guidance.mp4", CV_FOURCC('F','M','P','4'), params.frame_rate, img_final.size());
  for (int i = 0; i < 5.0f / params.frame_rate) {
    writer.write(img_final);
  }

  std::vector<cv::Mat> img_arr_sr, img_arr_mcts;
  std::vector<std::string> str_arr_sr, str_arr_mcts;

  while (current_state_sr.loc_node != goal_idx) {
    Action action = sr->getBestAction(current_state);
    extended_model.takeAction(current_state_sr, action, unused_reward, next_state, unused_terminal,
                              unused_depth_count, rng, unused_time_loss, unused_utility_loss, frame_vector);

    BOOST_FOREACH(State &state, frame_vector) {
      img_sr = base_image.clone();
      model.drawState(state, img_sr);
      std::stringstream ss;
      ss << actions[counter];
      img_arr_sr.push_back(img_sr);
      str_arr_sr.push_back(ss.str());
      /* cv::putText(img, ss.str(), cv::Point2f(22, img.rows-22), CV_FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 3); */
      /* boost::this_thread::sleep(boost::posix_time::milliseconds(0.25f * 1000.0f / params.frame_rate)); */
    }

    current_state_sr = next_state;
  }

  img_arr_sr.push_back(img_sr);
  str_arr_sr.push_back("Episode Finished");

  while (current_state_mcts.loc_node != goal_idx) {
    Action action = mcts->getBestAction(current_state);
    extended_model.takeAction(current_state_mcts, action, unused_reward, next_state, unused_terminal,
                              unused_depth_count, rng, unused_time_loss, unused_utility_loss, frame_vector);

    BOOST_FOREACH(State &state, frame_vector) {
      img_mcts = base_image.clone();
      model.drawState(state, img_mcts);
      std::stringstream ss;
      ss << actions[counter];
      img_arr_mcts.push_back(img_mcts);
      str_arr_mcts.push_back(ss.str());
      /* cv::putText(img, ss.str(), cv::Point2f(22, img.rows-22), CV_FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 3); */
      /* boost::this_thread::sleep(boost::posix_time::milliseconds(0.25f * 1000.0f / params.frame_rate)); */
    }

    current_state_mcts = next_state;
  }

  img_arr_mcts.push_back(img_mcts);
  str_arr_mcts.push_back("Episode Finished");

  int max_counter = std::max(img_arr_sr.size(), img_arr_mcts.size());


  return 0;
}
