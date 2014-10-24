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

  PersonModel model(graph, map, 12, 0, 10, 0, 10, 0, 1.0, 0.5);

  float unused_reward, unused_time_loss, unused_utility_loss;
  bool unused_terminal;
  int unused_depth_count;
  std::vector<State> unused_frame_vector;

  boost::shared_ptr<RNG> rng(new RNG(1));
  
  // Original state - state 1.
  State s0;
  s0.graph_id = 8;
  s0.precision = 1.0f;
  s0.from_graph_node = 8;
  s0.direction = 0; // computeNextDirection(0, 8, 9, graph);
  s0.robot_gave_direction = false;
  s0.robots.resize(2);
  s0.robots[0].graph_id = 8;
  s0.robots[0].destination = 6;
  s0.robots[0].precision = 0.0f;
  s0.robots[0].other_graph_node = 8;
  s0.robots[1].graph_id = 9;
  s0.robots[1].destination = 10;
  s0.robots[1].precision = 0.0f;
  s0.robots[1].other_graph_node = 9;

  State s1;
  model.takeAction(s0, Action(ASSIGN_ROBOT, 8, 0), unused_reward, s1, unused_terminal, unused_depth_count, rng,
                   unused_time_loss, unused_utility_loss, unused_frame_vector);

  cv::Mat img1 = base_image.clone();
  model.drawState(s1, img1);
  cv::imwrite("intro_state_1.png", img1);

  // State 2 - After taking action AssignRobot(2, 9)
  State s2;
  model.takeAction(s1, Action(ASSIGN_ROBOT, 9, 0), unused_reward, s2, unused_terminal, unused_depth_count, rng,
                   unused_time_loss, unused_utility_loss, unused_frame_vector);
  cv::Mat img2 = base_image.clone();
  model.drawState(s2, img2);
  cv::imwrite("intro_state_2.png", img2);

  // State 3 - After taking action GuidePerson(1,9)
  State s3 = s2;
  Action a3;
  a3.type = GUIDE_PERSON;
  a3.at_graph_id = 8;
  a3.guide_graph_id = 9;

  cv::Mat img3 = base_image.clone();
  model.drawState(s3, img3);
  model.drawAction(s3, a3, img3);
  cv::imwrite("intro_state_3.png", img3);

  // State 4 - After taking action ReleaseRobot(1)
  State s4;
  model.takeAction(s3, Action(GUIDE_PERSON, 8, 0), unused_reward, s4, unused_terminal, unused_depth_count, rng,
                   unused_time_loss, unused_utility_loss, unused_frame_vector);
  cv::Mat img4 = base_image.clone();
  model.drawState(s4, img4);
  cv::imwrite("intro_state_4.png", img4);

  // State 5 - After taking action Wait
  State s5;
  model.takeAction(s4, Action(WAIT, 0, 0), unused_reward, s5, unused_terminal, unused_depth_count, rng,
                   unused_time_loss, unused_utility_loss, unused_frame_vector);
  cv::Mat img5 = base_image.clone();
  s5.precision = 1.0f;
  model.drawState(s5, img5);
  cv::imwrite("intro_state_5.png", img5);
  
  // State 6 - After taking action lead Person
  State s6 = s5;
  Action a6;
  a6.type = LEAD_PERSON;
  a6.at_graph_id = 9;
  a6.guide_graph_id = 12;
  cv::Mat img6 = base_image.clone();
  model.drawState(s6, img6);
  model.drawAction(s6, a6, img6);
  cv::imwrite("intro_state_6.png", img6);

  // State 7 - terminal after taking WAIT.
  State s7;
  std::cout << s6.robots[0].graph_id << " " << s6.robots[0].precision << " " << s6.robots[0].other_graph_node << std::endl;
  model.takeAction(s6, Action(LEAD_PERSON, 9, 12), unused_reward, s7, unused_terminal, unused_depth_count, rng,
                   unused_time_loss, unused_utility_loss, unused_frame_vector);
  s7.precision = 1.0f;
  cv::Mat img7 = base_image.clone();
  model.drawState(s7, img7);
  cv::imwrite("intro_state_7.png", img7);

  // Action a2 = a1;
  // a2.type = LEAD_PERSON;

  // // Draw the images and save them to file.
  // cv::Mat img1 = base_image.clone();
  // , img2 = base_image.clone();
  // model.drawState(s1, img1);
  // model.drawAction(s1, a1, img1);

  // model.drawState(s1, img2);
  // model.drawAction(s1, a2, img2);

  // cv::imwrite("intro_state_1", img1);
  // cv::imwrite("intro_state_2", img2);
  // cv::imwrite("intro_state_3", img3);
  // cv::imwrite("intro_state_4", img4);
  // cv::imwrite("intro_state_5", img5);
  
  return 0;
}
