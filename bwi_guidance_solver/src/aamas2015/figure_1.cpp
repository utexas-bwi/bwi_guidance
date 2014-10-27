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

  PersonModel::Params params;
  MotionModel::Ptr motion_model(new MotionModel(graph, 
                                                params.avg_robot_speed / map.info.resolution,
                                                1.0f / map.info.resolution)); 
  std::vector<int> robot_home_base;
  robot_home_base.push_back(11);
  robot_home_base.push_back(12);
  TaskGenerationModel::Ptr task_generation_model(new TaskGenerationModel(robot_home_base, graph, 1.0f));

  HumanDecisionModel::Ptr human_decision_model(new HumanDecisionModel(graph));

  PersonModel model(graph, map, 12, motion_model, human_decision_model, task_generation_model, params);

  float unused_reward, unused_time_loss, unused_utility_loss;
  bool unused_terminal;
  int unused_depth_count;
  std::vector<State> unused_frame_vector;

  boost::shared_ptr<RNG> rng(new RNG(0));
  
  // Original state - state 1.
  State current_state, next_state;
  current_state.loc_node = 8;
  current_state.loc_p = 0.0f;
  current_state.loc_v = 0;
  current_state.direction = 0; // computeNextDirection(0, 8, 9, graph);
  current_state.assist_type = NONE;
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

  model.takeAction(current_state, Action(ASSIGN_ROBOT, 0, 8), unused_reward, next_state, unused_terminal, 
                   unused_depth_count, rng, unused_time_loss, unused_utility_loss, unused_frame_vector);
  current_state = next_state;

  std::vector<Action> actions;
  actions.push_back(Action(DIRECT_PERSON, 0, 9));
  actions.push_back(Action(RELEASE_ROBOT, 0));
  actions.push_back(Action(ASSIGN_ROBOT, 1, 9));
  actions.push_back(Action(WAIT));
  actions.push_back(Action(LEAD_PERSON, 1, 12));
  actions.push_back(Action(WAIT));

  int counter = 0;
  while(true) { 
    std::cout << "Drawing state: " << current_state << std::endl;

    cv::Mat img = base_image.clone();
    model.drawState(current_state, img);
    cv::imwrite("intro_state_" + boost::lexical_cast<std::string>(counter + 1) + ".png", img);

    if (current_state.loc_node == 12) {
      break;
    }

    std::cout << "  Taking action: " << actions[counter] << std::endl;

    model.takeAction(current_state, actions[counter], unused_reward, next_state, unused_terminal, 
                     unused_depth_count, rng, unused_time_loss, unused_utility_loss, unused_frame_vector);
    current_state = next_state;

    ++counter;
  }
  
  return 0;
}
