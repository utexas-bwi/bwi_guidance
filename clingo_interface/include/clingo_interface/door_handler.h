/**
 * \file  door_handler.h
 * \brief  A wrapper around navfn and costmap_2d that determines if a door is 
 *         open or not, and calculates navigation targets while approaching a 
 *         door and going through a door
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
 * $ Id: 05/06/2013 11:24:01 AM piyushk $
 *
 **/

#ifndef DOOR_HANDLER_WW75RJPS
#define DOOR_HANDLER_WW75RJPS

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <clingo_interface/structures.h>
#include <topological_mapper/map_loader.h>
#include <topological_mapper/map_utils.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/GetPlan.h>

namespace clingo_interface {

  class DoorHandler {

    public:

      DoorHandler (std::string map_file, std::string door_file, std::string location_file) {
        readDoorFile(door_file, doors_);
        readLocationFile(location_file, locations_, location_map_);
        mapper_.reset(new topological_mapper::MapLoader(map_file));
        nav_msgs::OccupancyGrid grid;
        mapper_->getMap(grid);
        info_ = grid.info;

        ros::NodeHandle n;
        make_plan_client_ = 
          n.serviceClient<nav_msgs::GetPlan>("move_base/NavfnROS/make_plan"); //TODO maybe not need Navfn here?
        make_plan_client_.waitForExistence();
        std::cout << "Make plan server AVAILABLE" << std::endl; 
      }

      bool isDoorOpen(size_t idx) {

        if (idx > doors_.size()) {
          return false;
        }

        topological_mapper::Point2f start_pt, goal_pt;
        float start_yaw, goal_yaw;

        start_pt = doors_[idx].approach_points[0];
        goal_pt = doors_[idx].approach_points[1];
        start_yaw = doors_[idx].approach_yaw[0];
        goal_yaw = doors_[idx].approach_yaw[1];

        nav_msgs::GetPlan srv;
        geometry_msgs::PoseStamped &start = srv.request.start, &goal = srv.request.goal;
        start.header.frame_id = goal.header.frame_id = "/map";
        start.header.stamp = goal.header.stamp = ros::Time::now();

        start.pose.position.x = start_pt.x;
        start.pose.position.y = start_pt.y;
        start.pose.position.z = 0;
        start.pose.orientation = tf::createQuaternionMsgFromYaw(start_yaw); 

        goal.pose.position.x = goal_pt.x;
        goal.pose.position.y = goal_pt.y;
        goal.pose.position.z = 0;
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw);
        srv.request.tolerance = 0.5 + 1e-6;

        float min_distance = sqrt(pow(start_pt.x - goal_pt.x, 2) + pow(start_pt.y - goal_pt.y, 2));

        if (make_plan_client_.call(srv)) {
          if (srv.response.plan.poses.size() != 0) {
            // Valid plan received. Try rough check that plan distance seems reasonable
            float distance = 0;
            geometry_msgs::Point old_pt = srv.response.plan.poses[0].pose.position;
            for (size_t i = 1; i < srv.response.plan.poses.size(); ++i) {
              geometry_msgs::Point current_pt = srv.response.plan.poses[i].pose.position;
              distance += sqrt(pow(current_pt.x - old_pt.x, 2) + pow(current_pt.y - old_pt.y, 2));
              old_pt = current_pt;
            }
            std::cout << "checking door " << idx << ". valid plan received. min, actual: " << min_distance << " vs " << distance << std::endl;
            if (distance < 3 * min_distance) {
              return true;
            } else {
              return false; // returned path probably through some other door
            }
          } else {
            return false; // this is ok. it means the door is closed
          }
        } else {
          return false; // shouldn't be here. the service has failed
        }
      }

      float getPathCost(const topological_mapper::Point2f& start_pt, float start_yaw,
          const topological_mapper::Point2f& goal_pt, float goal_yaw) {
        nav_msgs::GetPlan srv;
        geometry_msgs::PoseStamped &start = srv.request.start, &goal = srv.request.goal;
        start.header.frame_id = goal.header.frame_id = "/map";
        start.header.stamp = goal.header.stamp = ros::Time::now();

        start.pose.position.x = start_pt.x;
        start.pose.position.y = start_pt.y;
        start.pose.position.z = 0;
        start.pose.orientation = tf::createQuaternionMsgFromYaw(start_yaw); 

        goal.pose.position.x = goal_pt.x;
        goal.pose.position.y = goal_pt.y;
        goal.pose.position.z = 0;
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw);
        srv.request.tolerance = 0.5 + 1e-6;

        if (make_plan_client_.call(srv)) {
          if (srv.response.plan.poses.size() != 0) {
            // Valid plan received. Try rough check that plan distance seems reasonable
            float distance = 0;
            geometry_msgs::Point old_pt = srv.response.plan.poses[0].pose.position;
            for (size_t i = 1; i < srv.response.plan.poses.size(); ++i) {
              geometry_msgs::Point current_pt = srv.response.plan.poses[i].pose.position;
              distance += sqrt(pow(current_pt.x - old_pt.x, 2) + pow(current_pt.y - old_pt.y, 2));
              old_pt = current_pt;
            }
            return distance;
          } else {
            return 0; // the 2 points not in same location. this function should not have been used
          }
        } else {
          return 0; // shouldn't be here. the service has failed
        }
      }

      bool getApproachPoint(size_t idx, 
          const topological_mapper::Point2f& current_location,
          topological_mapper::Point2f& point, float &yaw) {

        if (idx > doors_.size()) {
          return false;
        }

        for (size_t pt = 0; pt < 2; ++pt) {
          std::cout << getLocationIdx(doors_[idx].approach_names[pt]) << " vs " <<  getLocationIdx(current_location) << std::endl;
          if (getLocationIdx(doors_[idx].approach_names[pt]) == getLocationIdx(current_location)) {
            point = doors_[idx].approach_points[pt];
            yaw = doors_[idx].approach_yaw[pt];
            return true;
          }
        }

        /* The door is not approachable from the current location */
        return false;
      }

      bool getThroughDoorPoint(size_t idx, 
          const topological_mapper::Point2f& current_location,
          topological_mapper::Point2f& point, float& yaw) {

        if (idx > doors_.size()) {
          return false;
        }

        for (size_t pt = 0; pt < 2; ++pt) {
          if (getLocationIdx(doors_[idx].approach_names[pt]) == getLocationIdx(current_location)) {
            point = doors_[idx].approach_points[1 - pt];
            yaw = M_PI + doors_[idx].approach_yaw[1 - pt];
            return true;
          }
        }

        return false;
      }

      bool isPointBesideDoor(const topological_mapper::Point2f& current_location,
          float threshold, size_t idx) {
        for (size_t pt = 0; pt < 2; ++pt) {
          if (cv::norm(doors_[idx].approach_points[pt] - current_location) < 
              threshold) {
            return true;
          }
        }
      }

      size_t getLocationIdx(const topological_mapper::Point2f& current_location) {

        topological_mapper::Point2f grid = topological_mapper::toGrid(current_location, info_);
        size_t map_idx = MAP_IDX(info_.width, (int) grid.x, (int) grid.y);
        if (map_idx > location_map_.size()) {
          return (size_t) -1;
        }
        return (size_t) location_map_[map_idx];

      }

      inline size_t getLocationIdx(const std::string& loc_str) const {
        for (size_t i = 0; i < locations_.size(); ++i) {
          if (locations_[i] == loc_str) {
            return i;
          }
        }
        return (size_t)-1;
      }

      inline size_t getDoorIdx(const std::string& door_str) const {
        for (size_t i = 0; i < doors_.size(); ++i) {
          if (doors_[i].name == door_str) {
            return i;
          }
        }
        return (size_t)-1;
      }

      inline std::string getLocationString(size_t idx) const {
        if (idx >= locations_.size())
          return "";
        return locations_[idx];
      }

      inline std::string getDoorString(size_t idx) const {
        if (idx >= doors_.size())
          return "";
        return doors_[idx].name;
      }

      inline size_t getNumDoors() const {
        return doors_.size();
      }

    private:

      std::vector<clingo_interface::Door> doors_;
      std::vector<std::string> locations_;
      std::vector<int32_t> location_map_;

      std::string door_yaml_file_;
      std::string location_file_;
      std::string map_file_;
      boost::shared_ptr <topological_mapper::MapLoader> mapper_;
      nav_msgs::MapMetaData info_;

      ros::ServiceClient make_plan_client_;

      // boost::shared_ptr <navfn::NavfnROS> navfn_;
      // boost::shared_ptr <costmap_2d::Costmap2DROS> costmap_;
      // boost::shared_ptr<clingo_interface::CostmapDoorPlugin> door_plugin_;

  }; /* DoorHandler */
  
} /* clingo_interface */

#endif /* end of include guard: DOOR_HANDLER_WW75RJPS */
