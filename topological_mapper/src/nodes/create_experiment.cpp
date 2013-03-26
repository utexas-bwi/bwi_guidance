/**
 * \file  create_experiment.cpp
 * \brief  Create a single experiment in a series of experiments for the bwi_web
 *         experiments
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 *
 * Copyright (c) 2013, UT Austin

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *
 * $ Id: 03/23/2013 09:07:41 PM piyushk $
 *
 **/

#include <topological_mapper/topological_mapper.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

enum state {
  START_LOC,
  START_YAW,
  GOAL_LOC,
  ROBOTS,
  PRINT
} global_state = START_LOC;

cv::Point clicked_pt;
bool increment_state = false;
bool new_robot_available = false;

topological_mapper::Point2f toMap(const cv::Point &pt, 
    const nav_msgs::MapMetaData& info) {

  topological_mapper::Point2f real_loc;
  real_loc.x = info.origin.position.x + info.resolution * pt.x;
  real_loc.y = info.origin.position.y + info.resolution * pt.y;
  return real_loc;
}

void mouseCallback(int event, int x, int y, int, void*) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    clicked_pt = cv::Point(x, y);
    if (global_state != ROBOTS) {
      increment_state = true;
    } else {
      new_robot_available = true;
    }
  }
}

void getShortestPath(topological_mapper::Graph &graph, size_t start_idx,
    size_t goal_idx, std::vector<size_t> &path_from_goal) {

  // Perform Dijakstra from start_idx
  std::vector<topological_mapper::Graph::vertex_descriptor> 
    p(boost::num_vertices(graph));
  std::vector<double> d(boost::num_vertices(graph));
  topological_mapper::Graph::vertex_descriptor s = 
    boost::vertex(start_idx, graph);

  boost::property_map<topological_mapper::Graph, boost::vertex_index_t>::type 
      indexmap = boost::get(boost::vertex_index, graph);
  boost::property_map<
    topological_mapper::Graph, 
    double topological_mapper::Edge::*
  >::type weightmap = boost::get(&topological_mapper::Edge::weight, graph);
  boost::dijkstra_shortest_paths(graph, s, &p[0], &d[0], weightmap, indexmap, 
                            std::less<double>(), boost::closed_plus<double>(), 
                            (std::numeric_limits<double>::max)(), 0,
                            boost::default_dijkstra_visitor());

  // Look up the parent chain from the goal vertex to the start vertex
  path_from_goal.clear();

  topological_mapper::Graph::vertex_descriptor g = 
    boost::vertex(goal_idx, graph);
  while (indexmap[p[g]] != start_idx) {
    path_from_goal.push_back(indexmap[p[g]]);
    g = p[g];
  }

}

int main(int argc, char** argv) {

  if (argc != 3) {
    std::cerr << "USAGE: " << argv[0] 
        << " <yaml-map-file> <yaml-graph-file>" << std::endl;
    return -1;
  }

  topological_mapper::TopologicalMapper mapper(argv[1]);
  topological_mapper::Graph graph;
  nav_msgs::MapMetaData info;
  mapper.getMapInfo(info);
  topological_mapper::readGraphFromFile(argv[2], info, graph);

  cv::Mat image;
  mapper.drawMap(image);
  topological_mapper::drawGraph(image, graph);

  cv::namedWindow("Display", CV_WINDOW_AUTOSIZE);
  cv::setMouseCallback("Display", mouseCallback, 0);

  std::stringstream ss;
  topological_mapper::Point2f start_pt;
  cv::Point start_image;
  start_pt.x = 0; start_pt.y = 0;
  std::vector<size_t> robot_idx;
  double yaw;

  while (true) {
    cv::imshow("Display", image);
    unsigned char c = cv::waitKey(10);
    if (c == 27) {
      return 0;
    } else if (c == 'p' && global_state == ROBOTS) {
      increment_state = true;
    }

    if (increment_state) {
      topological_mapper::Point2f map_pt = toMap(clicked_pt, info);
      switch(global_state) {
        case START_LOC:
          start_pt = map_pt; start_image = clicked_pt;
          ss << "  - start_x: " << map_pt.x << std::endl;
          ss << "    start_y: " << map_pt.y << std::endl;
          cv::circle(image, clicked_pt, 10, cv::Scalar(255,0,0), 2);
          global_state = START_YAW;
          break;
        case START_YAW:
          yaw = atan2(map_pt.y - start_pt.y, map_pt.x - start_pt.x);
          ss << "    start_yaw: " 
             << yaw
             << std::endl;
          cv::line(image, start_image, 
              start_image + cv::Point(20 * cosf(yaw), 20 * sinf(yaw)),
              cv::Scalar(255,0,0), 1, 4);
          global_state = GOAL_LOC;
          break;
        case GOAL_LOC:
          ss << "    ball_x: " << map_pt.x << std::endl;
          ss << "    ball_y: " << map_pt.y << std::endl;
          cv::circle(image, clicked_pt, 10, cv::Scalar(0,255,0), 2);
          global_state = ROBOTS;
          break;
        case ROBOTS: {
          ss << "    path: blah" << std::endl;
          std::cout << ss.str();
          global_state = PRINT;
          break;
        }
        default:
          return 0;
      }
      increment_state = false;
    } else if (new_robot_available) {
      // figure out how to push to robot_idx here;
      std::cout << "new robot here" << std::endl;
      new_robot_available = false;
    }
  }

  return 0;
}




  
