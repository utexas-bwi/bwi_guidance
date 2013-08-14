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
#include <topological_mapper/map_utils.h>
#include <bwi_exp1/base_robot_positioner.h>

using namespace bwi_exp1;

class DataCollectionRobotPositioner : public bwi_exp1::BaseRobotPositioner {
  public:
    DataCollectionRobotPositioner(boost::shared_ptr<ros::NodeHandle>& nh);
    virtual ~DataCollectionRobotPositioner() {}

    virtual void startExperimentInstance(int instance_number) {
        # Teleport all the robots to their respective positions
        self.path = experiment_data['path']
        self.robots_in_instance = \
                [{'id': p['id'], 'path_position': i} 
                 for i, p in enumerate(self.path) if p['robot']]
        self.extra_robots = experiment_data['extra_robots']
        self.robot_locations = [None] * len(self.robots)
        self.robot_images = [None] * len(self.robots)
        for i, robot in enumerate(self.robots):
            if i < len(self.robots_in_instance):
                # This robit is being used in this instance
                # Find a position for it
                path_position = self.robots_in_instance[i]['path_position']

                # Get position and orientation of robot from C++ based service
                req = PositionRobotRequest()
                req.from_id = -1
                req.to_id = -1
                req.at_id = self.robots_in_instance[i]['id']
                if path_position != 0:
                    req.from_id = self.path[path_position - 1]['id']
                    arriving_from = self.get_location_at_graph_id(req.from_id)
                else:
                    req.from_pt.x = start_x
                    req.from_pt.y = start_y
                    req.from_pt.z = 0
                    arriving_from = [start_x, start_y]
                if path_position != len(self.path) - 1:
                    req.to_id = self.path[path_position + 1]['id']
                    going_to = self.get_location_at_graph_id(req.to_id)
                else:
                    req.to_pt.x = ball_x
                    req.to_pt.y = ball_y
                    req.to_pt.z = 0
                    going_to = [ball_x, ball_y]

                resp = self.position_robot(req)
                robot_loc = [resp.loc.x, resp.loc.y]
                robot_yaw = resp.yaw

                # Get orientation of the arrow on the screen
                # If the point in the graph is too close, select the next point
                destination_distance = distance(going_to, robot_loc)
                path_position = path_position + 1
                while destination_distance < 3:
                    if path_position != len(self.path) - 1:
                        going_to = self.get_location_at_graph_id(
                            self.path[path_position + 1]['id'])
                    else:
                        going_to = [ball_x, ball_y]
                    destination_distance = distance(going_to, robot_loc)
                    path_position = path_position + 1

                destination_yaw = orientation(going_to, robot_loc)
                change_in_yaw = destination_yaw - robot_yaw
                change_in_yaw = normalize_angle(change_in_yaw) 

                # Compute the arrow based on direction
                robot_image = produce_directed_arrow(self.arrow, change_in_yaw)

            elif (i - len(self.robots_in_instance)) < len(self.extra_robots): 
                # This is an extra (distraction) robot. Move it into place
                extra_robot = self.extra_robots[i-len(self.robots_in_instance)]
                
                robot_loc = [extra_robot['loc_x'], extra_robot['loc_y']]
                robot_yaw = extra_robot['yaw']
                robot_image = self.image_none

            else: # Move a robot not used in the experiment
                robot_loc = self.robots[i]['default_loc']
                robot_yaw = 0
                robot_image = self.image_none

            # Perform the actual teleport and image update
            robot_pose = get_pose_msg_from_2d(*robot_loc, yaw=robot_yaw)
            result = self.teleport_entity(self.robots[i]['id'], robot_pose)
            self.robot_images[i] = robot_image
            self.robot_locations[i] = robot_loc


    }

    virtual void finalizeExperimentInstance(int instance_number) = 0; {
        for robot in self.robots:
            robot_pose = get_pose_msg_from_2d(*robot['default_loc'], yaw=0)
            self.teleport_entity(robot['id'], robot_pose) 
    }

    virtual void odometryCallback(const nav_msgs::Odometry::ConstPtr) {
        # check if the person is near a robot
        for i, robot in enumerate(self.robots):
            if i < len(self.robots_in_instance):
                # This is non-distraction robot that is a part of the instance
                robot_loc = self.robot_locations[i]
                distance_from_robot = distance(loc, robot_loc)
                if distance_from_robot < 3.0:
                    self.robot_image_publisher.update(robot['id'],
                                                      self.robot_images[i])
                else:
                    self.robot_image_publisher.update(robot['id'], 
                                                      self.image_none)


    }



};

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

  if (topological_mapper::getMagnitude(yawmid_pt) < 0.1) {
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
