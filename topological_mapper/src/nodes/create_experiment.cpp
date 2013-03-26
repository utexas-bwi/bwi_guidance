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
  START_IDX,
  GOAL_LOC,
  GOAL_IDX,
  ROBOTS,
  STOP
} global_state = START_LOC;

cv::Point clicked_pt;
cv::Point mouseover_pt;
bool increment_state = false;
bool new_robot_available = false;
int highlight_idx = -1;

topological_mapper::Point2f toMap(const cv::Point &pt, 
    const nav_msgs::MapMetaData& info) {

  topological_mapper::Point2f real_loc;
  real_loc.x = info.origin.position.x + info.resolution * pt.x;
  real_loc.y = info.origin.position.y + info.resolution * pt.y;
  return real_loc;
}

void circleIdx(cv::Mat& image, topological_mapper::Graph& graph, 
    size_t idx, cv::Scalar color = cv::Scalar(0, 0, 255)) {

  topological_mapper::Graph::vertex_descriptor vd = boost::vertex(idx, graph);
  topological_mapper::Point2f &location = graph[vd].location;
  cv::circle(image, cv::Point(location.x, location.y), 7, color, 2); 
  
}

void highlightIdx(cv::Mat& image, topological_mapper::Graph& graph, 
    size_t idx, cv::Scalar color = cv::Scalar(0, 255, 0)) {

  topological_mapper::Graph::vertex_descriptor vd = boost::vertex(idx, graph);
  topological_mapper::Point2f &location = graph[vd].location;
  cv::circle(image, cv::Point(location.x, location.y), 5, color, -1); 
  
}

void mouseCallback(int event, int x, int y, int, void*) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    clicked_pt = cv::Point(x, y);
    if (global_state != ROBOTS) {
      increment_state = true;
    } else {
      new_robot_available = true;
    }
  } else if (event == cv::EVENT_MOUSEMOVE) {
    mouseover_pt = cv::Point(x, y);
  }
}

size_t getClosestIdOnGraph(const cv::Point &point, 
    topological_mapper::Graph &graph) {

  topological_mapper::Graph::vertex_iterator vi, vend;
  int count = 0;
  for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
    topological_mapper::Point2f location = graph[*vi].location;
    if (fabs(point.x - location.x) <= 5 && fabs(point.y - location.y) <=5) {
      return count;
    }
    count++;
  }
  return -1;
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
  path_from_goal.push_back(goal_idx);

  topological_mapper::Graph::vertex_descriptor g = 
    boost::vertex(goal_idx, graph);
  while (indexmap[p[g]] != start_idx) {
    path_from_goal.push_back(indexmap[p[g]]);
    g = p[g];
  }
  path_from_goal.push_back(start_idx);

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

  cv::namedWindow("Display", CV_WINDOW_AUTOSIZE);
  cv::setMouseCallback("Display", mouseCallback, 0);

  topological_mapper::Point2f map_start, map_goal;
  double yaw = 0;
  map_start.x = 0; map_start.y = 0;
  map_goal.x = 0; map_goal.y = 0;
  cv::Point pxl_start, pxl_goal;
  size_t start_idx = 0, goal_idx = 0;
  std::vector<size_t> path_idx;
  std::vector<size_t> robot_idx;

  std::stringstream ss;
  while (true) {

    mapper.drawMap(image,0,0);
    topological_mapper::drawGraph(image, graph, 0, 0, false);

    // Clicks
    if (global_state > START_LOC) {
      cv::circle(image, pxl_start, 10, cv::Scalar(255,0,0), 2);
    }
    if (global_state > START_YAW) {
      cv::line(image, pxl_start, 
          pxl_start + cv::Point(20 * cosf(yaw), 20 * sinf(yaw)),
          cv::Scalar(0,0,0), 1, 4);
    }
    if (global_state > START_IDX) {
      highlightIdx(image, graph, start_idx, cv::Scalar(255, 0, 0));
    }
    if (global_state > GOAL_LOC) {
      cv::circle(image, pxl_goal, 10, cv::Scalar(0,255,0), 2);
    }
    if (global_state > GOAL_IDX) {
      for (size_t t = 0; t < path_idx.size(); ++t) {
        highlightIdx(image, graph, path_idx[t], cv::Scalar(255, 0, 0));
      }
      highlightIdx(image, graph, goal_idx, cv::Scalar(0, 255, 0));
      for (size_t t = 0; t < robot_idx.size(); ++t) {
        circleIdx(image, graph, robot_idx[t], cv::Scalar(0, 0, 255));
      }
    }

    // Highlight
    if (global_state == START_IDX || global_state == GOAL_IDX || 
        global_state == ROBOTS) {
      size_t highlight_idx = getClosestIdOnGraph(mouseover_pt, graph);
      if (highlight_idx != (size_t) -1) {
        highlightIdx(image, graph, highlight_idx); 
      }
    }

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
          map_start = map_pt; pxl_start = clicked_pt;
          ss << "  - start_x: " << map_pt.x << std::endl;
          ss << "    start_y: " << map_pt.y << std::endl;
          global_state = START_YAW;
          break;
        case START_YAW:
          yaw = atan2(map_pt.y - map_start.y, map_pt.x - map_start.x);
          ss << "    start_yaw: " 
             << yaw
             << std::endl;
          global_state = START_IDX;
          break;
        case START_IDX:
          start_idx = getClosestIdOnGraph(mouseover_pt, graph);
          if (start_idx != (size_t) -1) {
            global_state = GOAL_LOC;
          }
          break;
        case GOAL_LOC:
          map_goal = map_pt; pxl_goal = clicked_pt; 
          ss << "    ball_x: " << map_pt.x << std::endl;
          ss << "    ball_y: " << map_pt.y << std::endl;
          global_state = GOAL_IDX;
          break;
        case GOAL_IDX:
          goal_idx = getClosestIdOnGraph(mouseover_pt, graph);
          if (goal_idx != (size_t) -1) {
            getShortestPath(graph, start_idx, goal_idx, path_idx);
            global_state = ROBOTS;
          }
          break;
        case ROBOTS: {
          // Push out the path array
          ss << "    path:" << std::endl;
          for (size_t i = path_idx.size() - 1; i < path_idx.size(); --i) {
            ss << "      - id: " << path_idx[i] << std::endl;
            if (std::find(robot_idx.begin(), robot_idx.end(), path_idx[i]) != 
                robot_idx.end()) {
              ss << "        robot: yes" << std::endl;
            } else {
              ss << "        robot: no" << std::endl;
            }
          }
          ss << "    max_duration: 120 #seconds" << std::endl;
          std::cout << ss.str();
          global_state = STOP;
          break;
        }
        default:
          return 0;
      }
      increment_state = false;
    } else if (new_robot_available) {
      size_t idx = getClosestIdOnGraph(mouseover_pt, graph);
      if (idx != (size_t)-1) {
        if (std::find(path_idx.begin(), path_idx.end(), idx) != path_idx.end()) {
          robot_idx.push_back(idx);
        }
      }
      new_robot_available = false;
    }
  }

  return 0;
}




  
