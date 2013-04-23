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
#include <topological_mapper/map_inflator.h>
#include <topological_mapper/map_utils.h>
#include <topological_mapper/point_utils.h>
#include <topological_mapper/graph.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

enum State {
  START_LOC = 0,
  START_YAW = 1,
  GOAL_LOC = 2,
  ROBOTS = 3,
  EXTRA_ROBOT_LOC = 4,
  EXTRA_ROBOT_YAW = 5,
  STOP = 6
} global_state = START_LOC;

cv::Point clicked_pt;
cv::Point mouseover_pt;
bool increment_state = false;
bool new_robot_available = false;
int highlight_idx = -1;

void findStartAndGoalIdx(cv::Point start_pxl, cv::Point goal_pxl, 
    topological_mapper::Graph &graph, size_t &start_idx, size_t &goal_idx) {

  cv::Vec2f start(start_pxl.x, start_pxl.y);
  cv::Vec2f goal(goal_pxl.x, goal_pxl.y);

  boost::property_map<topological_mapper::Graph, boost::vertex_index_t>::type 
      indexmap = boost::get(boost::vertex_index, graph);

  float start_fitness = std::numeric_limits<float>::max();
  float goal_fitness = std::numeric_limits<float>::max();

  topological_mapper::Graph::vertex_iterator vi, vend;
  for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
    cv::Vec2f loc(graph[*vi].location.x, graph[*vi].location.y);
    topological_mapper::Graph::adjacency_iterator ai, aend;
    for (boost::tie(ai, aend) = boost::adjacent_vertices(
          (topological_mapper::Graph::vertex_descriptor)*vi, graph); 
        ai != aend; ++ai) {
      cv::Vec2f loc2(graph[*ai].location.x, graph[*ai].location.y);

      // Improve start idx as necessary
      float start_distance = 
          topological_mapper::minimumDistanceToLineSegment(loc, loc2, start);
      if (start_distance < start_fitness) {
        start_fitness = start_distance;
        if (cv::norm(loc - goal) < cv::norm(loc2 - goal)) {
          start_idx = indexmap[*vi];
        } else {
          start_idx = indexmap[*ai];
        }
      }

      // Improve goal idx as necessary
      float goal_distance = 
          topological_mapper::minimumDistanceToLineSegment(loc, loc2, goal);
      if (goal_distance < goal_fitness) {
        goal_fitness = goal_distance;
        if (cv::norm(loc - start) < cv::norm(loc2 - start)) {
          goal_idx = indexmap[*vi];
        } else {
          goal_idx = indexmap[*ai];
        }
      }

    }
  }

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
    if (global_state < ROBOTS) {
      increment_state = true;
    } else {
      new_robot_available = true;
    }
  } else if (event == cv::EVENT_MOUSEMOVE) {
    mouseover_pt = cv::Point(x, y);
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
  std::vector<cv::Point> robot_pxls;
  std::vector<cv::Point> extra_robot_pxls;
  std::vector<topological_mapper::Point2f> extra_robot_locations;
  std::vector<float> extra_robot_yaw;

  std::stringstream ss;
  while (true) {

    mapper.drawMap(image,0,0);
    topological_mapper::drawGraph(image, graph, 0, 0, true);

    // Clicks
    if (global_state > START_LOC) {
      cv::circle(image, pxl_start, 10, cv::Scalar(255,0,0), 2);
    }
    if (global_state > START_YAW) {
      cv::line(image, pxl_start, 
          pxl_start + cv::Point(20 * cosf(yaw), 20 * sinf(yaw)),
          cv::Scalar(255,0,0), 1, 4);
    }
    if (global_state > GOAL_LOC) {
      cv::circle(image, pxl_goal, 10, cv::Scalar(0,255,0), 2);
      highlightIdx(image, graph, start_idx, cv::Scalar(255, 0, 0));
      for (size_t t = 0; t < path_idx.size(); ++t) {
        highlightIdx(image, graph, path_idx[t], cv::Scalar(255, 0, 0));
      }
      highlightIdx(image, graph, goal_idx, cv::Scalar(0, 255, 0));
      for (size_t t = 0; t < robot_idx.size(); ++t) {
        circleIdx(image, graph, robot_idx[t], cv::Scalar(0, 0, 255));
      }
      for (size_t t = 0; t < robot_pxls.size(); ++t) {
        cv::Point &robot = robot_pxls[t];
        cv::circle(image, robot, 10, cv::Scalar(0,0,255), 2);
      }
      for (size_t t = 0; t < extra_robot_pxls.size(); ++t) {
        cv::Point &robot = extra_robot_pxls[t];
        cv::circle(image, robot, 10, cv::Scalar(0,0,255), 2);
      }
      for (size_t t = 0; t < extra_robot_yaw.size(); ++t) {
        cv::Point &robot = extra_robot_pxls[t];
        float yaw = -extra_robot_yaw[t];
        cv::line(image, robot, 
            robot + cv::Point(20 * cosf(yaw), 20 * sinf(yaw)),
            cv::Scalar(0,0,255), 1, 4);
      }
    }

    // Highlight
    if (global_state == ROBOTS) {
      size_t highlight_idx = 
          topological_mapper::getClosestIdOnGraph(mouseover_pt, graph);
      if (highlight_idx != (size_t) -1) {
        highlightIdx(image, graph, highlight_idx); 
      }
    } else if (global_state == EXTRA_ROBOT_LOC) {
      cv::circle(image, mouseover_pt, 10, cv::Scalar(0,255, 0), 2);
    } else if (global_state == EXTRA_ROBOT_YAW) {
      cv::Point &prev_pt = extra_robot_pxls[extra_robot_pxls.size() - 1];
      float yaw = atan2(mouseover_pt.y - prev_pt.y, mouseover_pt.x - prev_pt.x);
      cv::line(image, prev_pt, 
          prev_pt + cv::Point(20 * cosf(yaw), 20 * sinf(yaw)),
          cv::Scalar(0,255,0), 1, 4);
    }

    cv::imshow("Display", image);

    unsigned char c = cv::waitKey(10);
    if (c == 27) {
      return 0;
    } else if (c == 'n' && global_state >= ROBOTS) {
      increment_state = true;
    }

    if (increment_state) {
      topological_mapper::Point2f map_pt = 
          topological_mapper::toMap(clicked_pt, info);
      switch(global_state) {
        case START_LOC:
          map_start = map_pt; pxl_start = clicked_pt;
          ss << "- start_x: " << map_pt.x << std::endl;
          ss << "  start_y: " << map_pt.y << std::endl;
          global_state = START_YAW;
          break;
        case START_YAW:
          yaw = atan2(map_pt.y - map_start.y, map_pt.x - map_start.x);
          ss << "  start_yaw: " 
             << yaw
             << std::endl;
          global_state = GOAL_LOC;
          break;
        case GOAL_LOC:
          map_goal = map_pt; pxl_goal = clicked_pt; 
          ss << "  ball_x: " << map_pt.x << std::endl;
          ss << "  ball_y: " << map_pt.y << std::endl;
          findStartAndGoalIdx(pxl_start, pxl_goal, graph, start_idx, goal_idx);
          topological_mapper::getShortestPath(graph, start_idx, goal_idx, path_idx);
          global_state = ROBOTS;
          break;
        case ROBOTS: 
          ss << "  path:" << std::endl;
          for (size_t i = path_idx.size() - 1; i < path_idx.size(); --i) {
            ss << "    - id: " << path_idx[i] << std::endl;
            std::vector<size_t>::iterator robot_it =
              std::find(robot_idx.begin(), robot_idx.end(), path_idx[i]);
            if (robot_it != robot_idx.end()) {
              ss << "      robot: yes" << std::endl;
            } else {
              ss << "      robot: no" << std::endl;
            }
          }
          global_state = EXTRA_ROBOT_LOC;
          break;
        case EXTRA_ROBOT_LOC:
        case EXTRA_ROBOT_YAW:
          // Push out the path array
          ss << "  extra_robots:";
          if (extra_robot_locations.size() == 0) {
            ss << " []" << std::endl;
          } else {
            ss << std::endl;
            for (size_t i = 0; i < extra_robot_locations.size(); ++i) {
              ss << "    - loc_x: " << extra_robot_locations[i].x << std::endl;
              ss << "      loc_y: " << extra_robot_locations[i].y << std::endl;
              ss << "      yaw: " << extra_robot_yaw[i] << std::endl;
            }
          }
          ss << "  max_duration: 300 #seconds" << std::endl;
          std::cout << ss.str();
          global_state = STOP;
          break;
        default:
          return 0;
      }
      increment_state = false;
    } else if (new_robot_available) {
      if (global_state == ROBOTS) {
        size_t idx = 
            topological_mapper::getClosestIdOnGraph(mouseover_pt, graph);
        if (idx != (size_t)-1) {
          if (std::find(path_idx.begin(), path_idx.end(), idx) != path_idx.end()) {
            robot_idx.push_back(idx);
          }
        }
      } else if (global_state == EXTRA_ROBOT_LOC) {
        topological_mapper::Point2f map_pt = 
            topological_mapper::toMap(clicked_pt, info);
        extra_robot_pxls.push_back(clicked_pt);
        extra_robot_locations.push_back(map_pt);
        global_state = EXTRA_ROBOT_YAW;
      } else if (global_state == EXTRA_ROBOT_YAW) {
        cv::Point &prev_pt = extra_robot_pxls[extra_robot_pxls.size() - 1];
        float yaw = -atan2(clicked_pt.y - prev_pt.y, clicked_pt.x - prev_pt.x);
        extra_robot_yaw.push_back(yaw);
        global_state = EXTRA_ROBOT_LOC;
      }
      new_robot_available = false;
    }
  }

  return 0;
}




  
