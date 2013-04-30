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
#include <topological_mapper/map_utils.h>
#include <topological_mapper/graph.h>
#include <topological_mapper/point_utils.h>

#include <bwi_msgs/PositionRobot.h>

#include <opencv2/highgui/highgui.hpp>

ros::ServiceServer position_server;
std::string map_file, graph_file;
double search_distance;
topological_mapper::Graph graph;
nav_msgs::MapMetaData info;
nav_msgs::OccupancyGrid map;
bool debug = false;
double robot_radius;
double robot_padding;
boost::shared_ptr<topological_mapper::MapLoader> mapper;

topological_mapper::Point2f getLocationFromMapPose(
    const geometry_msgs::Point32 &pt) {
  return topological_mapper::toGrid(topological_mapper::Point2f(pt.x, pt.y), 
      info); 
}

bool position(bwi_msgs::PositionRobot::Request &req,
    bwi_msgs::PositionRobot::Response &resp) {

  topological_mapper::Point2f from(0,0), at(0,0), to(0,0);
  if (req.from_id != -1) {
    from = topological_mapper::getLocationFromGraphId(req.from_id, graph);
  } else {
    from = getLocationFromMapPose(req.from_pt);
  }
  topological_mapper::Point2f from_map = 
    topological_mapper::toMap(from, info);

  at = topological_mapper::getLocationFromGraphId(req.at_id, graph);
  topological_mapper::Point2f at_map = 
    topological_mapper::toMap(at, info);

  if (req.to_id != -1) {
    to = topological_mapper::getLocationFromGraphId(req.to_id, graph);
  } else {
    to = getLocationFromMapPose(req.to_pt);
  }

  // Figure out if you want to stay on the outside angle or not
  bool use_outside_angle = true;
  float yaw1 = -atan2((to - at).y, (to - at).x);
  float yaw2 = -atan2((from - at).y, (from - at).x);

  topological_mapper::Point2f yaw1_pt(cosf(yaw1),sinf(yaw1));
  topological_mapper::Point2f yaw2_pt(cosf(yaw2),sinf(yaw2));
  topological_mapper::Point2f yawmid_pt = 0.5 * (yaw1_pt + yaw2_pt);

  if (cv::norm(yawmid_pt) < 0.1) {
    use_outside_angle = false;
  }

  size_t y_test = at.y - search_distance / info.resolution;
  float location_fitness = -1;
  topological_mapper::Point2f test_coords;
  topological_mapper::Point2f map_coords;

  while(y_test < at.y + search_distance / info.resolution) {
    size_t x_test = at.x - search_distance / info.resolution;
    while (x_test < at.x + search_distance / info.resolution) {

      topological_mapper::Point2f test(x_test, y_test);
      
      // Check if x_test, y_test is free.
      size_t map_idx = MAP_IDX(info.width, x_test, y_test);
      if (map.data[map_idx] != 0) {
        x_test++;
        continue;
      }

      // Check if it is on the outside or not
      if (use_outside_angle) {
        if ((from + to - 2 * at).dot(test - at) > 0) {
          x_test++;
          continue;
        }
      }

      topological_mapper::Point2f test_loc(x_test, y_test);

      float dist1 = 
        topological_mapper::minimumDistanceToLineSegment(from, at, test_loc);
      float dist2 = 
        topological_mapper::minimumDistanceToLineSegment(at, to, test_loc);
      float fitness = std::min(dist1, dist2);

      if (fitness > location_fitness) {
        test_coords = test_loc;
        map_coords = topological_mapper::toMap(test_loc, info);
        resp.loc.x = map_coords.x;
        resp.loc.y = map_coords.y;
        location_fitness = fitness;
      }

      x_test++; 
    }
    y_test++;
  }

  // Calculate yaw
  yaw1 = atan2(resp.loc.y - at_map.y, resp.loc.x - at_map.x); 
  yaw2 = atan2(resp.loc.y - from_map.y, resp.loc.x - from_map.x);

  yaw1_pt = topological_mapper::Point2f(cosf(yaw1),sinf(yaw1));
  yaw2_pt = topological_mapper::Point2f(cosf(yaw2),sinf(yaw2));
  yawmid_pt = 0.5 * (yaw1_pt + yaw2_pt);

  resp.yaw = atan2f(yawmid_pt.y, yawmid_pt.x); 

  if (debug) {
    cv::Mat image;
    mapper->drawMap(image, map);

    cv::circle(image, from, 5, cv::Scalar(255, 0, 0), -1);
    cv::circle(image, at, 5, cv::Scalar(255, 0, 255), 2);
    cv::circle(image, to, 5, cv::Scalar(0, 0, 255), -1);
    cv::circle(image, test_coords, 5, cv::Scalar(0, 255, 0), 2);

    cv::circle(image, test_coords + 
        topological_mapper::Point2f(20 * cosf(resp.yaw), 20 * sinf(resp.yaw)), 
        3, cv::Scalar(0, 255, 0), -1);

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
  ROS_INFO_STREAM("Reading graph: " << graph_file);

  ros::param::param<bool>("~debug", debug, false);
  ros::param::param<double>("~robot_radius", robot_radius, 0.25);
  ros::param::param<double>("~robot_padding", robot_padding, 0.1);
  ros::param::param<double>("~search_distance", search_distance, 0.75);
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
