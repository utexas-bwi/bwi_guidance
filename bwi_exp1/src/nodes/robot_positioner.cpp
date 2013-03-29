/**
 * \file  robot_posistioner.cpp
 * \brief  Given a map of the world and its graphical representation, this node
 *         determines where to place a robot next to a given node when we wish
 *         to navigate a person through that node
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
 * $ Id: 03/27/2013 05:13:58 PM piyushk $
 *
 **/

#include <ros/ros.h>
#include <topological_mapper/map_loader.h>
#include <topological_mapper/map_inflator.h>
#include <topological_mapper/graph.h>

#include <bwi_msgs/PositionRobot.h>

#include <opencv2/highgui/highgui.hpp>

ros::ServiceServer position_server;
std::string map_file, graph_file;
topological_mapper::Graph graph;
nav_msgs::MapMetaData info;
nav_msgs::OccupancyGrid map;
bool debug = false;
double robot_radius;
double robot_padding;
boost::shared_ptr<topological_mapper::MapLoader> mapper;

cv::Vec2f toPxl(const cv::Vec2f &pt) {
  return cv::Vec2f(
      (pt[0] - info.origin.position.x) / info.resolution, 
      (pt[1] - info.origin.position.y) / info.resolution);
}

cv::Vec2f toMap(const cv::Vec2f &pt) { 
  return cv::Vec2f(
      info.origin.position.x + info.resolution * pt[0], 
      info.origin.position.y + info.resolution * pt[1]);
}

cv::Vec2f getLocationFromGraphId(int idx) {
  topological_mapper::Graph::vertex_descriptor i = boost::vertex(idx, graph);
  return cv::Vec2f(graph[i].location.x, graph[i].location.y);
}

cv::Vec2f getLocationFromMapPose(const geometry_msgs::Point32 &pt) {
  return toPxl(cv::Vec2f(pt.x, pt.y)); 
}

float minimumDistanceToLineSegment(cv::Vec2f v, cv::Vec2f w, cv::Vec2f p) {
  // Return minimum distance between line segment vw and point p
  const float l2 = cv::norm(w-v);  // i.e. |w-v| -  avoid a sqrt
  if (l2 == 0.0) return cv::norm(p-v);   // v == w case
  // Consider the line extending the segment, parameterized as v + t (w - v).
  // We find projection of point p onto the line. 
  // It falls where t = [(p-v) . (w-v)] / |w-v|^2
  const float t = (p - v).dot(w - v) / (l2 * l2);
  if (t < 0.0) return cv::norm(p - v);       // Beyond the 'v' end of the segment
  else if (t > 1.0) return cv::norm(p - w);  // Beyond the 'w' end of the segment
  const cv::Vec2f projection = v + t * (w - v);  // Projection falls on the segment
  return cv::norm(p - projection);
}

bool position(bwi_msgs::PositionRobot::Request &req,
    bwi_msgs::PositionRobot::Response &resp) {

  cv::Vec2f from(0,0), at(0,0), to(0,0);
  if (req.from_id != -1) {
    from = getLocationFromGraphId(req.from_id);
  } else {
    from = getLocationFromMapPose(req.from_pt);
  }
  cv::Vec2f from_map = toMap(from);

  at = getLocationFromGraphId(req.at_id);
  cv::Vec2f at_map = toMap(at);

  if (req.to_id != -1) {
    to = getLocationFromGraphId(req.to_id);
  } else {
    to = getLocationFromMapPose(req.to_pt);
  }

  // Figure out if you want to stay on the outside angle or not
  bool use_outside_angle = true;
  float yaw1 = -atan2(to[1] - at[1], to[0] - at[0]);
  float yaw2 = -atan2(from[1] - at[1], from[0] - at[0]);

  cv::Vec2f yaw1_pt(cosf(yaw1),sinf(yaw1));
  cv::Vec2f yaw2_pt(cosf(yaw2),sinf(yaw2));
  cv::Vec2f yawmid_pt = (yaw1_pt + yaw2_pt) / 2;

  if (cv::norm(yawmid_pt) < 0.1) {
    use_outside_angle = false;
  }

  float y_test = at[1] - 1.0 / info.resolution;
  float location_fitness = -1;
  cv::Vec2f test_coords;
  cv::Vec2f map_coords;

  while(y_test < at[1] + 1.0 / info.resolution) {
    float x_test = at[0] - 1.0 / info.resolution;
    while (x_test < at[0] + 1.0 / info.resolution) {
      
      // Check if x_test, y_test is free.
      size_t map_idx = MAP_IDX(info.width, x_test, y_test);
      if (map.data[map_idx] != 0) {
        x_test++;
        continue;
      }

      // Check if it is on the outside or not
      if (use_outside_angle) {
        if ((from + to - 2*at).dot(cv::Vec2f(x_test, y_test) - at) > 0) {
          x_test++;
          continue;
        }
      }

      cv::Vec2f test_loc(x_test, y_test);

      float dist1 = minimumDistanceToLineSegment(from, at, test_loc);
      float dist2 = minimumDistanceToLineSegment(at, to, test_loc);
      std::cout << x_test << " " << y_test << " " << dist1 << " " << dist2 << std::endl;
      float fitness = std::min(dist1, dist2);

      if (fitness > location_fitness) {
        test_coords = test_loc;
        map_coords = toMap(test_loc);
        resp.loc.x = map_coords[0];
        resp.loc.y = map_coords[1];
        location_fitness = fitness;
      }

      x_test++; 
    }
    y_test++;
  }

  // Calculate yaw
  yaw1 = atan2(resp.loc.y - at_map[1], resp.loc.x - at_map[0]); 
  yaw2 = atan2(resp.loc.y - from_map[1], resp.loc.x - from_map[0]);

  yaw1_pt = cv::Vec2f(cosf(yaw1),sinf(yaw1));
  yaw2_pt = cv::Vec2f(cosf(yaw2),sinf(yaw2));
  yawmid_pt = (yaw1_pt + yaw2_pt) / 2;

  /* if (cv::norm(yawmid_pt) < 0.1) { */
    resp.yaw = atan2f(yawmid_pt[1], yawmid_pt[0]); 
  // } else {
  //   resp.yaw = yaw1;
  // }

  if (debug) {
    cv::Mat image;
    mapper->drawMap(image, map);

    cv::circle(image, cv::Point(from[0],from[1]), 5, cv::Scalar(255, 0, 0), -1);
    cv::circle(image, cv::Point(at[0],at[1]), 5, cv::Scalar(255, 0, 255), 2);
    cv::circle(image, cv::Point(to[0],to[1]), 5, cv::Scalar(0, 0, 255), -1);
    cv::circle(image, cv::Point(test_coords[0],test_coords[1]), 5, cv::Scalar(0, 255, 0), 2);

    cv::circle(image, cv::Point(test_coords[0],test_coords[1]) + cv::Point(20*cosf(resp.yaw),20*sinf(resp.yaw)), 3, cv::Scalar(0, 255, 0), -1);

    cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);
    cv::imshow("Display window", image);
    cv::waitKey(0);
  }

  return true;

}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "robot_positioner");
  ros::NodeHandle node;

  if (!ros::param::has("~map_file")) {
    ROS_FATAL("Parameter ~map_file needs to be set.");
    return -1;
  }
  ros::param::get("~map_file", map_file);

  if (!ros::param::has("~graph_file")) {
    ROS_FATAL("Parameter ~graph_file needs to be set.");
    return -1;
  }
  ros::param::get("~graph_file", graph_file);

  ros::param::param<bool>("~debug", debug, false);
  ros::param::param<double>("~robot_radius", robot_radius, 0.3);
  ros::param::param<double>("~robot_padding", robot_padding, 0.05);
  ROS_INFO("Inflating map by %f.", robot_radius + robot_padding);

  nav_msgs::OccupancyGrid uninflated_map;
  mapper.reset(new topological_mapper::MapLoader(map_file));
  mapper->getMapInfo(info);
  mapper->getMap(uninflated_map);
 
  topological_mapper::inflateMap(robot_radius + robot_padding, 
      uninflated_map, map);
  topological_mapper::readGraphFromFile(graph_file, info, graph);

  position_server = node.advertiseService("position", position);

  ros::spin();

  return 0;
}
