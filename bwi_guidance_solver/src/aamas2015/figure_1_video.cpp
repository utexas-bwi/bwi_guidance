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

  std::string map_file = "package://bwi_guidance/maps/graph.yaml";
  std::string graph_file = "package://bwi_guidance/maps/graph_graph.yaml";

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

  MotionModel::Ptr motion_model(new MotionModel(graph,
                                                0.5f / map.info.resolution,
                                                1.0f / map.info.resolution));
  std::vector<int> robot_home_base;
  robot_home_base.push_back(11);
  robot_home_base.push_back(12);
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

  boost::shared_ptr<RNG> rng(new RNG(0));

  // Original state - state 1.
  ExtendedState current_state, next_state;
  current_state.loc_node = 8;
  current_state.loc_p = 0.0f;
  current_state.loc_prev = 8;
  current_state.assist_type = NONE;
  current_state.assist_loc = NONE;
  current_state.robots.resize(2);
  current_state.robots[0].loc_u = 8;
  current_state.robots[0].loc_p = 0.0f;
  current_state.robots[0].loc_v = 0;
  current_state.robots[0].tau_d = 6;
  current_state.robots[0].tau_u = 1.0f;
  current_state.robots[0].tau_t = 0.0f;
  current_state.robots[0].tau_total_task_time = 5.0f;
  current_state.robots[0].help_destination = NONE;
  current_state.robots[1].loc_u = 9;
  current_state.robots[1].loc_p = 0.0f;
  current_state.robots[1].loc_v = 0;
  current_state.robots[1].tau_d = 10;
  current_state.robots[1].tau_u = 1.0f;
  current_state.robots[1].tau_t = 0.0f;
  current_state.robots[1].tau_total_task_time = 5.0f;
  current_state.robots[1].help_destination = NONE;
  std::cout << "Original state: " << current_state << std::endl;

  extended_model.takeAction(current_state, Action(ASSIGN_ROBOT, 0, 8), unused_reward, next_state, unused_terminal,
                            unused_depth_count, rng, unused_time_loss, unused_utility_loss, frame_vector);
  current_state = next_state;
  current_state.prev_action = Action(WAIT);
  current_state.released_locations.clear();

  std::vector<Action> actions;
  actions.push_back(Action(DIRECT_PERSON, 0, 9));
  actions.push_back(Action(RELEASE_ROBOT, 0));
  actions.push_back(Action(ASSIGN_ROBOT, 1, 9));
  actions.push_back(Action(WAIT));
  actions.push_back(Action(LEAD_PERSON, 1, 12));
  actions.push_back(Action(WAIT));

  // Draw the original state and wait 8 seconds.
  cv::Mat img = base_image.clone();
  model.drawState(current_state, img);
  cv::imshow("out", img);
  std::cin.ignore();
  boost::this_thread::sleep(boost::posix_time::milliseconds(15000.0f));

  int counter = 0;
  while(true) {

    std::cout << "Drawing state: " << current_state << std::endl;

    std::cout << "  Unrestricted Actions: " << std::endl;
    std::vector<Action> available_actions;
    model.getAllActions(current_state, available_actions);
    BOOST_FOREACH(Action &action, available_actions) {
      std::cout << "    " << action << std::endl;
    }

    std::cout << "  Restricted Actions: " << std::endl;
    extended_model.getAllActions(current_state, available_actions);
    BOOST_FOREACH(Action &action, available_actions) {
      std::cout << "    " << action << std::endl;
    }

    // img = base_image.clone();
    // model.drawState(current_state, img);
    // cv::imwrite("intro_state_" + boost::lexical_cast<std::string>(counter + 1) + ".png", img);

    if (current_state.loc_node == 12) {
      break;
    }

    std::cout << "  Taking action: " << actions[counter] << std::endl;

    extended_model.takeAction(current_state, actions[counter], unused_reward, next_state, unused_terminal,
                              unused_depth_count, rng, unused_time_loss, unused_utility_loss, frame_vector);

    BOOST_FOREACH(State &state, frame_vector) {
      img = base_image.clone();
      model.drawState(state, img);
      std::stringstream ss;
      ss << actions[counter];
      cv::putText(img, ss.str(), cv::Point2f(22, img.rows-22), CV_FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 3);
      cv::imshow("out", img);
      boost::this_thread::sleep(boost::posix_time::milliseconds(0.25f * 1000.0f / params.frame_rate));
    }

    if (actions[counter].type != WAIT) {
      boost::this_thread::sleep(boost::posix_time::milliseconds(4000.0f));
    }

    current_state = next_state;

    //std::cin.ignore();

    ++counter;
  }

  return 0;
}
