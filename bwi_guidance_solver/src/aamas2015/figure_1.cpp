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

  PersonModel model(graph, map, 12);
  
  // Setup state and action to draw.
  State s1;
  s1.graph_id = 9;
  s1.precision = 1.0f;
  s1.from_graph_node = 8;
  s1.direction = computeNextDirection(0, 8, 9, graph);
  s1.robot_gave_direction = false;
  s1.robots.resize(1);
  s1.robots[0].graph_id = 9;
  s1.robots[0].destination = -1;
  s1.robots[0].precision = 0.0f;
  s1.robots[0].other_graph_node = 9;
  s1.in_use_robots.resize(1);
  s1.in_use_robots[0].robot_id = 0;
  s1.in_use_robots[0].destination = 9;
  s1.in_use_robots[0].reached_destination = true;

  Action a1;
  a1.type = GUIDE_PERSON;
  a1.at_graph_id = 9;
  a1.guide_graph_id = 12;

  Action a2 = a1;
  a2.type = LEAD_PERSON;

  // Draw the images and save them to file.
  cv::Mat img1 = base_image.clone(), img2 = base_image.clone();
  model.drawState(s1, img1);
  model.drawAction(s1, a1, img1);

  model.drawState(s1, img2);
  model.drawAction(s1, a2, img2);

  cv::imwrite("intro_guide.png", img1);
  cv::imwrite("intro_lead.png", img2);
  
  return 0;
}
