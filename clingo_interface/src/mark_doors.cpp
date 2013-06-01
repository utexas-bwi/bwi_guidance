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

#include <fstream>

#include <topological_mapper/connected_components.h>
#include <topological_mapper/topological_mapper.h>
#include <topological_mapper/map_utils.h>
#include <clingo_interface/structures.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using topological_mapper::toGrid;

enum State {
  DOOR_PT_0 = 0,
  DOOR_PT_1 = 1,
  DOOR_PT_2 = 2,
  DOOR_PT_3 = 3,
  DOOR_APPROACH_0 = 4,
  DOOR_APPROACH_0_YAW = 5,
  DOOR_APPROACH_1 = 6,
  DOOR_APPROACH_1_YAW = 7,
  GET_DOOR_NAMES = 8,
  MARK_LOCATION = 9,
  STOP = 10
} global_state = DOOR_PT_0;

cv::Point clicked_pt, mouseover_pt;
bool increment_state = false;
void mouseCallback(int event, int x, int y, int, void*) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    clicked_pt = cv::Point(x, y);
    increment_state = true;
  } else if (event == cv::EVENT_MOUSEMOVE) {
    mouseover_pt = cv::Point(x, y);
  }
}

int main(int argc, char** argv) {

  if (argc < 4) {
    std::cerr << "USAGE: " << argv[0] 
        << " <yaml-map-file> <door-file> <location-file>" << std::endl;
    return -1;
  }

  std::string door_file(argv[2]);
  std::string location_file(argv[3]);

  topological_mapper::TopologicalMapper mapper(argv[1]);
  topological_mapper::Graph graph;
  nav_msgs::MapMetaData info;
  mapper.getMapInfo(info);

  cv::Mat image;

  cv::namedWindow("Display", CV_WINDOW_AUTOSIZE);
  cv::setMouseCallback("Display", mouseCallback, 0);

  std::vector<clingo_interface::Door> doors;
  clingo_interface::Door current_door;
  std::vector<int32_t> component_map;
  std::vector<std::string> locations;
  std::string current_location;

  while (true) {

    mapper.drawMap(image,0,0);

    /* Handle drawing door related information */
    if (global_state == DOOR_PT_1) {
      cv::circle(image, toGrid(current_door.corners[0], info), 
          2, cv::Scalar(0,0,255), -1);
    } 
    // if (global_state >= DOOR_PT_2 && global_state <= DOOR_PT_3) {
    //   cv::line(image, 
    //       toGrid(current_door.corners[0], info),
    //       toGrid(current_door.corners[1], info),
    //       cv::Scalar(0,0,255), 2, 8);
    // }
    // if (global_state == DOOR_PT_3) {
    //   cv::line(image, 
    //       toGrid(current_door.corners[1], info),
    //       toGrid(current_door.corners[2], info),
    //       cv::Scalar(0,0,255), 2, 8);
    // }
    if (global_state > DOOR_PT_1 && global_state <= DOOR_APPROACH_1_YAW) {
      cv::line(image, 
          toGrid(current_door.corners[0], info),
          toGrid(current_door.corners[1], info),
          cv::Scalar(0,0,255), 2, 8);
      // int num_points[1] = {4};
      // const cv::Point points[4] = {
      //   toGrid(current_door.corners[0], info),
      //   toGrid(current_door.corners[1], info),
      //   toGrid(current_door.corners[2], info),
      //   toGrid(current_door.corners[3], info)
      // };
      // const cv::Point* point_list[1] = {&points[0]};
      // cv::fillPoly(image, point_list, num_points, 1, cv::Scalar(0,0,255));
    }
    if (global_state > DOOR_APPROACH_0 && global_state <= DOOR_APPROACH_1_YAW) {
      cv::circle(image, toGrid(current_door.approach_points[0], info), 
          3, cv::Scalar(0,0,255), -1);
    }
    if (global_state > DOOR_APPROACH_0_YAW && global_state <= DOOR_APPROACH_1_YAW) {
      topological_mapper::Point2f current_approach_pt = 
        toGrid(current_door.approach_points[0], info);
      float current_approach_yaw = current_door.approach_yaw[0];
      cv::Point2f yaw_pt(cosf(current_approach_yaw), sinf(current_approach_yaw));
      yaw_pt = 10 * yaw_pt;
      yaw_pt = current_approach_pt + yaw_pt;
      cv::line(image, current_approach_pt, yaw_pt, cv::Scalar(0,0,255), 2, -1);
    }
    if (global_state > DOOR_APPROACH_1 && global_state <= DOOR_APPROACH_1_YAW) {
      cv::circle(image, toGrid(current_door.approach_points[1], info), 
          3, cv::Scalar(0,0,255), -1);
    }
    for (size_t i = 0; i < doors.size(); ++i) {
      cv::line(image, 
          toGrid(doors[i].corners[0], info),
          toGrid(doors[i].corners[1], info),
          cv::Scalar(0,0,0), 2, 8);
      /* int num_points[1] = {4}; */
      // const cv::Point points[4] = {
      //   toGrid(doors[i].corners[0], info),
      //   toGrid(doors[i].corners[1], info),
      //   toGrid(doors[i].corners[2], info),
      //   toGrid(doors[i].corners[3], info)
      // };
      // const cv::Point* point_list[1] = {&points[0]};
      // cv::fillPoly(image, point_list, num_points, 1, cv::Scalar(255,0,0));
      // 
      // cv::circle(image, toGrid(doors[i].approach_points[0], info), 
      //     3, cv::Scalar(255,0,0), -1);
      // 
      // topological_mapper::Point2f current_approach_pt = 
      //   toGrid(doors[i].approach_points[0], info);
      // float current_approach_yaw = doors[i].approach_yaw[0];
      // cv::Point2f yaw_pt(cosf(current_approach_yaw), sinf(current_approach_yaw));
      // yaw_pt = 10 * yaw_pt;
      // yaw_pt = current_approach_pt + yaw_pt;
      // cv::line(image, current_approach_pt, yaw_pt, cv::Scalar(255,0,0), 2, -1);
      // 
      // cv::circle(image, toGrid(doors[i].approach_points[1], info), 
      //     3, cv::Scalar(255,0,0), -1);
      // 
      // current_approach_pt = 
      //   toGrid(doors[i].approach_points[1], info);
      // current_approach_yaw = doors[i].approach_yaw[1];
      // yaw_pt = cv::Point2f(cosf(current_approach_yaw), sinf(current_approach_yaw));
      // yaw_pt = 10 * yaw_pt;
      // yaw_pt = current_approach_pt + yaw_pt;
      // cv::line(image, current_approach_pt, yaw_pt, cv::Scalar(255,0,0), 2, -1);
    }
    // for (size_t i = 0; i < locations.size(); ++i) {
    //   cv::circle(image, toGrid(locations[i].loc, info), 3, cv::Scalar(128,255,128), -1);
    // }

    cv::imshow("Display", image);

    unsigned char c = cv::waitKey(10);
    if (c == 27) {
      return 0;
    } else if (c == 'n' && global_state < GET_DOOR_NAMES) {
      /* Get door names */
      for (size_t d = 0; d < doors.size(); ++d) {
        cv::line(image, 
            toGrid(doors[d].corners[0], info),
            toGrid(doors[d].corners[1], info),
            cv::Scalar(255,0,0), 2, 8);
        for (size_t q = 0; q < doors.size(); ++q) {
          if (d == q)
            continue;
          cv::line(image, 
              toGrid(doors[q].corners[0], info),
              toGrid(doors[q].corners[1], info),
              cv::Scalar(0,0,0), 2, 8);
        }
        cv::imshow("Display", image);
        cv::waitKey(10);
        std::cout << "Enter door name: ";
        std::cin >> doors[d].name;
      }
      global_state = MARK_LOCATION;
    } else if (c == 'n') {
      component_map.resize(image.rows*image.cols);
      cv::Mat gray_image;
      cvtColor(image, gray_image, CV_RGB2GRAY);
      topological_mapper::ConnectedComponents cc(gray_image, component_map);
      size_t num_components = cc.getNumberComponents();
      std::cout << "Found " << num_components << " components." << std::endl;
      for (size_t comp = 0; comp < num_components; ++comp) {
        for (int j = 1; j < image.rows; ++j) {
          cv::Vec3b* image_row_j = image.ptr<cv::Vec3b>(j);
          for (int i = 0; i < image.cols; ++i) {
            size_t map_idx = MAP_IDX(image.cols, i, j);
            if (component_map[map_idx] == -1)
              continue;
            cv::Vec3b& pixel = image_row_j[i];
            /* std::cout << component_map[map_idx] << " "; */
            if ((size_t)component_map[map_idx] == comp) {
              pixel[0] = 255;
              pixel[1] = 0;
              pixel[2] = 0;
            } else {
              pixel[0] = 0;
              pixel[1] = 255;
              pixel[2] = 0;
            }
          }
        }
        cv::imshow("Display", image);
        cv::waitKey(10);
        std::cout << "Enter location name: ";
        std::cin >> current_location;
        locations.push_back(current_location);
      }
      global_state = STOP;
      increment_state = true;
    }

    if (increment_state) {
      topological_mapper::Point2f map_pt = 
          topological_mapper::toMap(clicked_pt, info);
      switch(global_state) {
        case DOOR_PT_0:
          current_door.corners[0] = map_pt;
          global_state = DOOR_PT_1;
          break;
        case DOOR_PT_1:
          current_door.corners[1] = map_pt;
          global_state = DOOR_APPROACH_0;
          break;
        case DOOR_PT_2:
          // current_door.corners[2] = map_pt;
          // global_state = DOOR_PT_3;
          break;
        case DOOR_PT_3:
          // current_door.corners[3] = map_pt;
          // global_state = DOOR_APPROACH_0;
          break;
        case DOOR_APPROACH_0:
          current_door.approach_points[0] = map_pt;
          global_state = DOOR_APPROACH_0_YAW;
          break;
        case DOOR_APPROACH_0_YAW:
          current_door.approach_yaw[0] = 
            atan2f((map_pt - current_door.approach_points[0]).y,
                   (map_pt - current_door.approach_points[0]).x);
          global_state = DOOR_APPROACH_1;
          break;
        case DOOR_APPROACH_1:
          current_door.approach_points[1] = map_pt;
          global_state = DOOR_APPROACH_1_YAW;
          break;
        case DOOR_APPROACH_1_YAW:
          current_door.approach_yaw[1] = 
            atan2f((map_pt - current_door.approach_points[1]).y,
                   (map_pt - current_door.approach_points[1]).x);
          doors.push_back(current_door);
          global_state = DOOR_PT_0;
          break;
        case STOP:
          std::stringstream door_ss, location_ss;
          for (size_t i = 0; i < doors.size(); ++i) {
            clingo_interface::Door &door = doors[i];
            door_ss << "- corners: [";
            for (size_t j = 0; j < 2; ++j) {
              door_ss << "[" << door.corners[j].x << ", " << 
                  door.corners[j].y << "]";
              if (j != 1) {
                door_ss << ", ";
              }
            }
            door_ss << "]" << std::endl;
            door_ss << "  approach:" << std::endl;
            for (size_t j = 0; j < 2; ++j) {
              topological_mapper::Point2f grid_pt = toGrid(door.approach_points[j], info);
              size_t map_idx = MAP_IDX(image.cols, (int) grid_pt.x, (int) grid_pt.y); 
              size_t loc_idx = component_map[map_idx];
              std::string from_loc = locations[loc_idx];
              door_ss << "    - from: " << from_loc << std::endl;
              door_ss << "      point: " << "[" << door.approach_points[j].x << ", " << 
                  door.approach_points[j].y << ", " << door.approach_yaw[j] 
                  << "]" << std::endl;
            }
            door_ss << "  name: " << door.name << std::endl;
          }
          std::ofstream door_fout(door_file.c_str());
          door_fout << door_ss.str();
          door_fout.close();

          location_ss << "locations:" << std::endl;
          for (size_t i = 0; i < locations.size(); ++i) {
            std::string &loc = locations[i];
            location_ss << "  - " << loc << std::endl;
          }
          location_ss << "data: [" << std::endl;
          for (size_t x = 0; x < component_map.size(); ++x) {
            location_ss << component_map[x];
            if (x != component_map.size() - 1)
              location_ss << ", ";
          }
          location_ss << "]" << std::endl;

          std::ofstream location_fout(location_file.c_str());
          location_fout << location_ss.str();
          location_fout.close();

          return 0;
      }
      increment_state = false;
    } 
  }

  return 0;
}




  
