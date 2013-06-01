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

#include <boost/shared_ptr.hpp>
#include <navfn/navfn_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <clingo_interface/structures.h>
#include <clingo_interface/costmap_door_plugin.h>

namespace clingo_interface {

  class DoorHandler {

    public:

      DoorHandler (tf::TransformListener &tf) {

        /* Initialize costmap and planner */
        costmap_.reset(new costmap_2d::Costmap2DROS("door_handler_costmap", tf)); 
        costmap_->pause();
        ROS_INFO("Costmap size: %d, %d", costmap_->getCostmap()->getSizeInCellsX(), costmap_->getCostmap()->getSizeInCellsY());
        navfn_.reset(new navfn::NavfnROS("door_handler_costmap", costmap_.get()));

        std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = 
          costmap_->getLayeredCostmap()->getPlugins();

        bool door_plugin_initalized = false;
        for (size_t i = 0; i < plugins->size(); ++i) {
          if ((*plugins)[i]->getName().find("door_plugin") != std::string::npos) {
            door_plugin_ = boost::static_pointer_cast<
              clingo_interface::CostmapDoorPlugin
              >((*plugins)[i]); 
            door_plugin_initalized = true;
            break;
          }
        }

        if (!door_plugin_initalized) {
          throw std::runtime_error("No CostmapDoorPlugin available in layered costmap");
        }

        costmap_->start();

      }

      bool isDoorOpen(size_t idx) {

        /* Close all other doors in the costmap. The current door should be 
         * getting updated through observations */
        for (size_t i = 0; i < door_plugin_->doors().size(); ++i) {
          if (i == idx) {
            door_plugin_->openDoor(i);
            continue;
          }
          door_plugin_->closeDoor(i);
        }

        // while (!door_plugin_->isCostmapCurrent()) {
        //   boost::this_thread::sleep( boost::posix_time::milliseconds(10));
        // }

        /* Now get the start and goal locations for this door */
        geometry_msgs::PoseStamped start_pose, goal_pose;
        start_pose.header.frame_id = goal_pose.header.frame_id = "/map"; //TODO
        start_pose.pose.position.x = door_plugin_->doors()[idx].approach_points[0].x;
        start_pose.pose.position.y = door_plugin_->doors()[idx].approach_points[0].y;
        goal_pose.pose.position.x = door_plugin_->doors()[idx].approach_points[1].x;
        goal_pose.pose.position.y = door_plugin_->doors()[idx].approach_points[1].y;
        start_pose.pose.position.z = goal_pose.pose.position.z = 0;
        start_pose.pose.orientation.x = start_pose.pose.orientation.y = 
          start_pose.pose.orientation.z = 0;
        start_pose.pose.orientation.w = 1;
        goal_pose.pose.orientation = start_pose.pose.orientation;

        /* Finally, see if a plan is possible */
        std::vector<geometry_msgs::PoseStamped> plan; // Irrelevant
        return navfn_->makePlan(start_pose, goal_pose, plan);
      }

      bool getApproachPoint(size_t idx, 
          const topological_mapper::Point2f& current_location,
          topological_mapper::Point2f& point, float &yaw) {

        /* Close all doors */
        for (size_t i = 0; i < door_plugin_->doors().size(); ++i) {
          door_plugin_->closeDoor(i);
        }

        // while (!door_plugin_->isCostmapCurrent()) {
        //   boost::this_thread::sleep( boost::posix_time::milliseconds(10));
        // }

        /* Setup variables */
        geometry_msgs::PoseStamped start_pose, goal_pose;
        start_pose.header.frame_id = goal_pose.header.frame_id = "/map"; //TODO
        start_pose.pose.position.x = current_location.x;
        start_pose.pose.position.y = current_location.y;
        start_pose.pose.position.z = goal_pose.pose.position.z = 0;
        start_pose.pose.orientation.x = start_pose.pose.orientation.y = 
          start_pose.pose.orientation.z = 0;
        start_pose.pose.orientation.w = 1;
        goal_pose.pose.orientation = start_pose.pose.orientation;
        std::vector<geometry_msgs::PoseStamped> plan; // Irrelevant

        /* Check against 1st approach point */
        goal_pose.pose.position.x = door_plugin_->doors()[idx].approach_points[0].x;
        goal_pose.pose.position.y = door_plugin_->doors()[idx].approach_points[0].y;
        if (navfn_->makePlan(start_pose, goal_pose, plan)) {
          point = door_plugin_->doors()[idx].approach_points[0];
          yaw = door_plugin_->doors()[idx].approach_yaw[0];
          return true;
        }

        /* Otherwise check against 2nd approach point */
        goal_pose.pose.position.x = door_plugin_->doors()[idx].approach_points[1].x;
        goal_pose.pose.position.y = door_plugin_->doors()[idx].approach_points[1].y;
        if (navfn_->makePlan(start_pose, goal_pose, plan)) {
          point = door_plugin_->doors()[idx].approach_points[1];
          yaw = door_plugin_->doors()[idx].approach_yaw[1];
          return true;
        }

        /* The door is not approachable from the current location */
        return false;
      }

      bool getThroughDoorPoint(size_t idx, 
          const topological_mapper::Point2f& current_location,
          topological_mapper::Point2f& point, float& yaw) {

        topological_mapper::Point2f approach_point;
        float approach_yaw;
        bool point_available = 
          getApproachPoint(idx, current_location, approach_point, approach_yaw);

        if (point_available) {
          if (approach_point == door_plugin_->doors()[idx].approach_points[0]) {
            point = door_plugin_->doors()[idx].approach_points[1];
            yaw = M_PI + door_plugin_->doors()[idx].approach_yaw[1];
          } else {
            point = door_plugin_->doors()[idx].approach_points[0];
            yaw = M_PI + door_plugin_->doors()[idx].approach_yaw[0];
          }
          yaw = atan2f(sinf(yaw), cosf(yaw));
          return true;
        }
        return false;
      }

      size_t getLocation(const topological_mapper::Point2f& current_location) {

        /* Close all doors */
        for (size_t i = 0; i < door_plugin_->doors().size(); ++i) {
          door_plugin_->closeDoor(i);
        }

        // while (!door_plugin_->isCostmapCurrent()) {
        //   boost::this_thread::sleep( boost::posix_time::milliseconds(10));
        // }

        /* Setup variables */
        geometry_msgs::PoseStamped start_pose, goal_pose;
        start_pose.header.frame_id = goal_pose.header.frame_id = "/map";
        start_pose.pose.position.x = current_location.x;
        start_pose.pose.position.y = current_location.y;
        start_pose.pose.position.z = goal_pose.pose.position.z = 0;
        start_pose.pose.orientation.x = start_pose.pose.orientation.y = 
          start_pose.pose.orientation.z = 0;
        start_pose.pose.orientation.w = 1;
        goal_pose.pose.orientation = start_pose.pose.orientation;
        std::vector<geometry_msgs::PoseStamped> plan; // Irrelevant

        /* Find a location that is still reachable. We are at that location */
        for (size_t i = 0; i < door_plugin_->locations().size(); ++i) {
          /* Check if we can reach this location. If so, then return this idx */
          goal_pose.pose.position.x = door_plugin_->locations()[i].loc.x;
          goal_pose.pose.position.y = door_plugin_->locations()[i].loc.y;
          if (navfn_->makePlan(start_pose, goal_pose, plan)) {
            return i;
          }
        }

        return -1;

      }

    private:

      boost::shared_ptr <navfn::NavfnROS> navfn_;
      boost::shared_ptr <costmap_2d::Costmap2DROS> costmap_;
      boost::shared_ptr<clingo_interface::CostmapDoorPlugin> door_plugin_;

  }; /* DoorHandler */
  
} /* clingo_interface */

#endif /* end of include guard: DOOR_HANDLER_WW75RJPS */
