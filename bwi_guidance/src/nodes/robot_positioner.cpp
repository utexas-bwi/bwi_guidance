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
#include <tf/transform_datatypes.h>
#include <bwi_mapper/map_utils.h>
#include <bwi_mapper/point_utils.h>
#include <bwi_guidance/base_robot_positioner.h>
#include <boost/foreach.hpp>

using namespace bwi_guidance;

class DataCollectionRobotPositioner : public bwi_guidance::BaseRobotPositioner {

  private:
    std::vector<cv::Mat> robot_images_;
    std::vector<float> robot_orientations_;
    size_t assigned_robots_;

    DCExperimentRobots experiment_robots_;

  public:
    DataCollectionRobotPositioner(boost::shared_ptr<ros::NodeHandle>& nh) :
        BaseRobotPositioner(nh), assigned_robots_(0) {

      std::string robot_file;
      ros::NodeHandle private_nh("~");
      if (!private_nh.getParam("experiment_robot_file", robot_file)) {
        ROS_FATAL_STREAM("RobotPosition: ~experiment_robot_file parameter " <<
            "required");
        exit(-1);
      }

      readDCExperimentRobotsFromFile(robot_file, experiment_robots_);
    }

    virtual ~DataCollectionRobotPositioner() {}

    virtual void startExperimentInstance(
        const std::string& instance_name) {

      boost::mutex::scoped_lock lock(robot_modification_mutex_);

      // Clear any old data
      robot_images_.clear();
      robot_orientations_.clear();
      assigned_robots_ = 0;

      const Instance& instance = 
        getInstance(experiment_, instance_name);
      const DCInstanceRobots& robots_in_instance = 
        getDCInstance(instance_name, experiment_robots_);

      // First assign robots in the path
      for (size_t path_idx = 0; path_idx < robots_in_instance.path.size();
          ++path_idx) {

        const PathPoint& path_point = robots_in_instance.path[path_idx];
        if (!path_point.robot_present) {
          continue;
        }

        // Argh! These locations are in pixels
        
        // At location
        bwi_mapper::Point2f at_loc = 
          bwi_mapper::getLocationFromGraphId(
              path_point.graph_id, graph_);

        // Coming from location
        bwi_mapper::Point2f from_loc(
            instance.start_loc.x,
            instance.start_loc.y);
        from_loc = bwi_mapper::toGrid(from_loc, map_info_);
        if (path_idx != 0) {
          const PathPoint& prev_path_point = 
            robots_in_instance.path[path_idx-1];
          from_loc = bwi_mapper::getLocationFromGraphId(
              prev_path_point.graph_id, graph_);
        }

        // Going to location
        bwi_mapper::Point2f to_loc(
            instance.ball_loc.x,
            instance.ball_loc.y);
        to_loc = bwi_mapper::toGrid(to_loc, map_info_);
        if (path_idx != robots_in_instance.path.size() - 1) {
          const PathPoint& next_path_point = robots_in_instance.path[path_idx+1];
          to_loc = bwi_mapper::getLocationFromGraphId(
              next_path_point.graph_id, graph_);
        }

        // Get pose
        geometry_msgs::Pose pose = positionRobot(from_loc, at_loc, to_loc);
        float robot_yaw = tf::getYaw(pose.orientation);

        // Get arrow direction
        bwi_mapper::Point2f robot_loc(pose.position.x, pose.position.y);
        robot_loc = bwi_mapper::toGrid(robot_loc, map_info_);
        float destination_distance = 
          bwi_mapper::getMagnitude(to_loc - robot_loc);
        size_t path_position = path_idx + 1;
        while (destination_distance < (3.0 / map_info_.resolution) && 
            path_position < robots_in_instance.path.size()) {

          const PathPoint& next_path_point = 
            robots_in_instance.path[path_position];
          to_loc = bwi_mapper::getLocationFromGraphId(
              next_path_point.graph_id, graph_);
          destination_distance = 
            bwi_mapper::getMagnitude(to_loc - robot_loc);
          ++path_position;
        }

        bwi_mapper::Point2f change_loc = to_loc - robot_loc;
        float destination_yaw = atan2(change_loc.y, change_loc.x);
        float change_in_yaw = destination_yaw - robot_yaw;
        cv::Mat robot_image;
        produceDirectedArrow(change_in_yaw, robot_image);

        // Teleport the robot and assign a direction
        std::string robot_id = default_robots_.robots[assigned_robots_].id;
        robot_locations_[robot_id] = pose;
        robot_images_.push_back(robot_image);
        robot_orientations_.push_back(destination_yaw);
        ++assigned_robots_;
      }

      // Assign robots not controlled by the path
      BOOST_FOREACH(const Location& location, robots_in_instance.robots) {
        std::string robot_id = default_robots_.robots[assigned_robots_].id;
        robot_locations_[robot_id] = 
          convert2dToPose(location.x, location.y, location.yaw);
        robot_images_.push_back(blank_image_);
        robot_orientations_.push_back(std::numeric_limits<float>::quiet_NaN());
        ++assigned_robots_;
      }
    }

    virtual void odometryCallback(const nav_msgs::Odometry::ConstPtr odom) {

      bwi_mapper::Point2f person_loc(
          odom->pose.pose.position.x,
          odom->pose.pose.position.y);

      boost::mutex::scoped_lock lock(robot_modification_mutex_);

      size_t count = 0;
      BOOST_FOREACH(const Robot& robot, default_robots_.robots) {
        if (count < assigned_robots_) {
          bwi_mapper::Point2f robot_loc(
            assigned_robot_locations_[robot.id].position.x,
            assigned_robot_locations_[robot.id].position.y
          );
          float distance = 
            bwi_mapper::getMagnitude(robot_loc - person_loc);
          if (distance < 3.0) {
            robot_screen_publisher_.updateImage(robot.id, robot_images_[count]);
            robot_screen_orientations_[robot.id] = robot_orientations_[count];
          } else {
            robot_screen_publisher_.updateImage(robot.id, blank_image_);
            robot_screen_orientations_[robot.id] = 
                std::numeric_limits<float>::quiet_NaN(); 
          }
        }
        ++count;
      }

    }

};

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "data_collection_robot_positioner");
  boost::shared_ptr<ros::NodeHandle> nh;
  nh.reset(new ros::NodeHandle());
  DataCollectionRobotPositioner rp(nh);
  rp.run();

  return 0;
}
