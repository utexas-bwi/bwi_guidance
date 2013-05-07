/**
 * \file  door_detector.h
 * \brief  A wrapper around navfn and costmap_2d that determines if a door is 
 *         open or not
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

#ifndef DOOR_DETECTOR_WW75RJPS
#define DOOR_DETECTOR_WW75RJPS

#include <fstream>
#include <topological_mapper/structures/point.h>
#include <topological_mapper/map_loader.h>
#include <topological_mapper/map_utils.h>
#include <boost/shared_ptr.hpp>
#include <navfn/navfn_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <yaml-cpp/yaml.h>

namespace clingo_helpers {

  class Door {
    public:
      topological_mapper::Point2f approach_points[2];
      float approach_yaw[2];
      topological_mapper::Point2f corners[4];
  };

  void readDoorFile (const std::string& filename, std::vector<Door>& doors) {
    std::ifstream fin(filename.c_str());
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);

    doors.clear();
    for (size_t i = 0; i < doc.size(); i++) {
      Door door;
      const YAML::Node &door_node = doc[i]["corners"];
      for (size_t j = 0; j < 4; ++j) {
        door_node[j][0] >> door.corners[j].x;
        door_node[j][1] >> door.corners[j].y;
      }
      const YAML::Node &approach_node = doc[i]["approach"];
      for (size_t j = 0; j < 2; ++j) {
        approach_node[j][0] >> door.approach_points[j].x; 
        approach_node[j][1] >> door.approach_points[j].y; 
        approach_node[j][2] >> door.approach_yaw[j]; 
      }
      doors.push_back(door);
    }
  }

  class DoorDetector {

    public:

      DoorDetector (boost::shared_ptr<topological_mapper::MapLoader> mapper, 
          const std::string& door_yaml_file) : mapper_(mapper) {

        /* Initialize costmap and planner */
        tf_.reset(new tf::TransformListener());
        costmap_.reset(new costmap_2d::Costmap2DROS("door_detector_costmap", 
            *tf_));
        navfn_.reset(new navfn::NavfnROS("door_detector_planner", 
            costmap_.get()));

        /* Initialize information about the doors */
        readDoorFile(door_yaml_file, doors_);

        nav_msgs::OccupancyGrid grid;
        mapper_->getMap(grid);
        info_ = grid.info;
        map_frame_id_ = "/map";
        //map_frame_id_ = grid.header.frame_id; //TODO
      }

      bool isDoorOpen(size_t idx) {

        /* Close all other doors in the costmap. The current door should be 
         * getting updated through observations */
        for (size_t i = 0; i < doors_.size(); ++i) {
          if (i == idx) {
            continue;
          }
          closeDoor(i);
        }

        /* Now get the start and goal locations for this door */
        geometry_msgs::PoseStamped start_pose, goal_pose;
        start_pose.header.frame_id = goal_pose.header.frame_id = map_frame_id_;
        start_pose.pose.position.x = doors_[idx].approach_points[0].x;
        start_pose.pose.position.y = doors_[idx].approach_points[0].y;
        goal_pose.pose.position.x = doors_[idx].approach_points[1].x;
        goal_pose.pose.position.y = doors_[idx].approach_points[1].y;
        start_pose.pose.position.z = goal_pose.pose.position.z = 0;
        start_pose.pose.orientation.x = start_pose.pose.orientation.y = 
          start_pose.pose.orientation.z = 0;
        start_pose.pose.orientation.w = 1;
        goal_pose.pose.orientation = start_pose.pose.orientation;

        /* Finally, see if a plan is possible */
        std::vector<geometry_msgs::PoseStamped> plan; // Irrelevant
        return navfn_->makePlan(start_pose, goal_pose, plan);
      }

      /* We need to close all other doors to figure out if a particular door 
       * can be navigated or not */
      void closeDoor(size_t idx) {
        
        /* From the original map, get the image section corresponding to this
           door and update the map */
        topological_mapper::Point2f image_pixels[4];
        for (size_t i = 0; i < 4; ++i) {
          image_pixels[i] = topological_mapper::toGrid(doors_[idx].corners[i], info_);
        }
        size_t min_x = -1, min_y = -1;
        size_t max_x = 0, max_y = 0;
        for (size_t i = 0; i < 4; ++i) {
          if (image_pixels[i].y < min_y) {
            min_y = image_pixels[i].y;
          }
          if (image_pixels[i].y > max_y) {
            max_y = image_pixels[i].y; 
          }
          if (image_pixels[i].x < min_x) {
            min_x = image_pixels[i].x;
          }
          if (image_pixels[i].x > max_x) {
            max_x = image_pixels[i].x; 
          }
        }
        size_t width = max_x - min_x;
        size_t height = max_y - min_y;

        cv::Mat image;
        mapper_->drawMap(image);
        cv::Mat sub_image = image(cv::Rect(min_x, min_y, width, height));

        /* Get transformed Point** from door pixels */
        int num_points[1] = {4};
        const cv::Point points[4] = {
          cv::Point(image_pixels[0].x - min_x, image_pixels[0].y - min_y),
          cv::Point(image_pixels[1].x - min_x, image_pixels[1].y - min_y),
          cv::Point(image_pixels[2].x - min_x, image_pixels[2].y - min_y),
          cv::Point(image_pixels[3].x - min_x, image_pixels[3].y - min_y),
        };
        const cv::Point* point_list[1] = {&points[0]};
        cv::fillPoly(sub_image, point_list, num_points, 1, cv::Scalar(0,0,0));

        /* Convert sub_image to Occupancy Grid and update map */
        nav_msgs::OccupancyGrid new_grid;
        new_grid.info.map_load_time = info_.map_load_time;
        new_grid.info.resolution = info_.resolution;
        new_grid.info.width = width;
        new_grid.info.height = height;
        new_grid.info.origin.orientation = info_.origin.orientation;
        new_grid.info.origin.position.x = 
            info_.origin.position.x + min_x * info_.resolution;
        new_grid.info.origin.position.y = 
            info_.origin.position.y + min_y * info_.resolution;
        new_grid.header.frame_id = map_frame_id_;
        new_grid.data.resize(height * width);

        /* Update the map */
        costmap_->updateStaticMap(new_grid);
      }

    private:

      boost::shared_ptr <topological_mapper::MapLoader> mapper_;
      boost::shared_ptr <navfn::NavfnROS> navfn_;
      boost::shared_ptr <costmap_2d::Costmap2DROS> costmap_;
      boost::shared_ptr <tf::TransformListener> tf_;

      std::vector<Door> doors_;
      nav_msgs::MapMetaData info_;
      std::string map_frame_id_;

  }; /* DoorDetector */
  
} /* clingo_helpers */

#endif /* end of include guard: DOOR_DETECTOR_WW75RJPS */
