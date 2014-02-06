#include <boost/foreach.hpp>
#include <bwi_mapper/graph.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_guidance_solver/person_model_iros14.h>
#include <opencv/highgui.h>
#include <ros/package.h>
#include <ros/ros.h>

using namespace bwi_guidance;

int main(int argc, const char *argv[]) {

  // std::string map_file = ros::package::getPath("bwi_guidance") +
  //   "/maps/map3.yaml";
  // std::string graph_file = ros::package::getPath("bwi_guidance") +
  //   "/maps/graph_map3.yaml";
  std::string map_file = ros::package::getPath("bwi_mapper") +
    "/maps/graph2.yaml";
  std::string graph_file = ros::package::getPath("bwi_mapper") +
    "/graph.yaml";

  bwi_mapper::MapLoader mapper(map_file);
  bwi_mapper::Graph graph;
  nav_msgs::OccupancyGrid map;
  mapper.getMap(map);
  bwi_mapper::readGraphFromFile(graph_file, map.info, graph);

  PersonModelIROS14 model(graph, map, 0);
  cv::Mat image;

  boost::mt19937 mt(0);
  boost::uniform_int<int> i(0, boost::num_vertices(graph) - 1);
  boost::uniform_real<float> u(0.0f, 1.0f);
  boost::poisson_distribution<int> p(2);
  URGenPtr generative_model_gen(new URGen(mt, u));
  UIGenPtr idx_gen(new UIGen(mt, i));
  PIGenPtr robot_goal_gen(new PIGen(mt, p));
  model.initializeRNG(idx_gen, generative_model_gen, robot_goal_gen);

  StateIROS14 s;
  s.graph_id = 9;
  s.precision = 0.0f;
  // s.robots.resize(1);
  // s.robots[0].graph_id = 9;
  // s.robots[0].destination = 9;
  // s.in_use_robots.resize(1);
  // s.in_use_robots[0].robot_id = 9;
  // s.in_use_robots[0].destination = 9;
  // s.in_use_robots[0].direction = -2;

  s.direction = 0;
  s.robot_gave_direction = false;

  s.direction = 1;
  s.robot_gave_direction = true;

  /* model.addRobots(s, 1); */
  model.setState(s);

  unsigned char c = 0;
  int count = 0;
  float reward;
  bool terminal = false;
  int depth_count;
  StateIROS14 state = s;
  std::map<StateIROS14, int> test_map;
  while(!terminal && c != 27) {
    mapper.drawMap(image, map);
    model.drawState(state, image);
    cv::imshow("out", image);
    std::vector<ActionIROS14> actions;
    model.getActionsAtState(s, actions);
    std::cout << "Actions: " << std::endl;
    BOOST_FOREACH(ActionIROS14 a, actions) {
      std::cout << a << " ";
    }
    std::cout << std::endl;
    // if (count == 0) {
    //   model.takeAction(ActionIROS14(ASSIGN_ROBOT, 3, DIR_UNASSIGNED), reward, state, terminal, depth_count);
    // } else if (count == 3) {
    //   model.takeAction(ActionIROS14(GUIDE_PERSON, 3, 43), reward, state, terminal, depth_count);
    // } else if (count == 5) {
    //   model.takeAction(ActionIROS14(RELEASE_ROBOT, 3, 0), reward, state, terminal, depth_count);
    // } else {
    //   std::cout << "got to this point" << std::endl;
      model.takeAction(ActionIROS14(), reward, state, terminal, depth_count);
    //   std::cout << "but next action fails" << std::endl;
    // }
    c = cv::waitKey(-1);
    std::cout << "Reward: " << reward << std::endl;
    std::cout << "  Depth: " << depth_count << std::endl;
    ++count;
  }

  return 0;
}
