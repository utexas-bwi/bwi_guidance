#include <bwi_mapper/graph.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_guidance_solver/person_model_iros14.h>
#include <opencv/highgui.h>
#include <ros/package.h>
#include <ros/ros.h>

using namespace bwi_guidance;

int main(int argc, const char *argv[]) {

  std::string map_file = ros::package::getPath("bwi_guidance") +
    "/maps/map3.yaml";
  std::string graph_file = ros::package::getPath("bwi_guidance") +
    "/maps/graph_map3.yaml";

  bwi_mapper::MapLoader mapper(map_file);
  bwi_mapper::Graph graph;
  nav_msgs::OccupancyGrid map;
  mapper.getMap(map);
  bwi_mapper::readGraphFromFile(graph_file, map.info, graph);

  PersonModelIROS14 model(graph, map, 0, 10);
  cv::Mat image;

  StateIROS14 s;
  s.graph_id = 5;
  s.direction = 0;

  boost::mt19937 mt(0);
  boost::uniform_int<int> i(0, boost::num_vertices(graph) - 1);
  boost::uniform_real<float> u(0.0f, 1.0f);
  boost::poisson_distribution<int> p(2);
  URGenPtr generative_model_gen(new URGen(mt, u));
  UIGenPtr idx_gen(new UIGen(mt, i));
  PIGenPtr robot_goal_gen(new PIGen(mt, p));
  model.initializeRNG(idx_gen, generative_model_gen, robot_goal_gen);

  model.setState(s);
  unsigned char c = 0;
  int count = 0;
  float reward;
  bool terminal = false;
  StateIROS14 state;
  while(!terminal && c != 27) {
    mapper.drawMap(image, map);
    model.drawCurrentState(image);
    cv::imshow("out", image);
    c = cv::waitKey(-1);
    if (count == 0) {
      model.takeAction(ActionIROS14(ASSIGN_ROBOT, 3, 43), reward, state, terminal);
    } else if (count == 5) {
      model.takeAction(ActionIROS14(RELEASE_ROBOT, 3, 0), reward, state, terminal);
    } else {
      model.takeAction(ActionIROS14(), reward, state, terminal);
    }
    std::cout << "Reward: " << reward << std::endl;
    ++count;
  }

  return 0;
}
