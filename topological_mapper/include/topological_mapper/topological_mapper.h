/**
 * \file  topological_mapper.h
 * \brief  Constructs the topological graph using the voronoi approximation
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
 * $ Id: 03/01/2013 12:06:45 PM piyushk $
 *
 **/

#ifndef TOPOLOGICAL_MAPPER_EA94RQRE
#define TOPOLOGICAL_MAPPER_EA94RQRE

#include <topological_mapper/connected_components.h>
#include <topological_mapper/voronoi_approximator.h>

namespace topological_mapper {

  class TopologicalMapper : public VoronoiApproximator {

    public:

      TopologicalMapper (const std::string &fname) :
        VoronoiApproximator(fname) {}

      void computeTopologicalGraph(double threshold, double critical_epsilon) {
        findVoronoiPoints(threshold);
        computeCriticalRegions(critical_epsilon);
      }

      void drawCriticalPoints(cv::Mat &image, 
          uint32_t orig_x = 0, uint32_t orig_y = 0) {

        for (size_t i = 0; i < critical_points_.size(); ++i) {
          VoronoiPoint &vp = critical_points_[i];
          cv::circle(image, 
              cv::Point(vp.x + orig_x, 
                        orig_y + inflated_map_.info.height - 1 - vp.y), 
              2, cv::Scalar(0,0,255), -1);
        }

        drawCriticalLines(image, orig_x, orig_y);
      }

      void drawConnectedComponents(cv::Mat &image,
          uint32_t orig_x = 0, uint32_t orig_y = 0) {

        // Figure out different colors - 1st color should always be black
        size_t num_colors = num_components_;
        component_colors_.resize(num_colors);
        component_colors_[0][0] = component_colors_[0][1] = component_colors_[0][2] = 0;
        for (size_t i = 1; i < num_colors; ++i) {
          component_colors_[i] = cv::Vec3b(rand() % 256, rand() % 256, rand() % 256);
        }

        // component 0 is obstacles + background. don't draw?

        // Now paint!
        for (uint32_t j = 1; j < map_resp_.map.info.height; ++j) {
          cv::Vec3b* image_row_j = image.ptr<cv::Vec3b>(j + orig_y);
          for (uint32_t i = 0; i < map_resp_.map.info.width; ++i) {
            size_t map_idx = MAP_IDX(map_resp_.map.info.width, i, j);
            if (component_number_[map_idx] == -1)
              continue;
            cv::Vec3b& pixel = image_row_j[i + orig_x];
            pixel[0] = component_colors_[component_number_[map_idx]][0];
            pixel[1] = component_colors_[component_number_[map_idx]][1];
            pixel[2] = component_colors_[component_number_[map_idx]][2];
            
          }
        }
      }

      void printCriticalPoints() {
        for (size_t i = 0; i < critical_points_.size(); ++i) {
          VoronoiPoint &vp = critical_points_[i];
          std::cout << " (" << vp.x << "," << vp.y << "): ";
          for (size_t j = 0; j < vp.basis_points.size(); ++j) {
            std::cout << " (" << vp.basis_points[j].x << "," << vp.basis_points[j].y << "," << vp.basis_points[j].distance_from_ref << "), ";
          }
          std::cout << std::endl;
        }
      }

      void drawOutput(cv::Mat &image) {
        VoronoiApproximator::drawOutput(image);
        drawMap(image, 2 * map_resp_.map.info.width);
        drawConnectedComponents(image, 2 * map_resp_.map.info.width);
        drawCriticalPoints(image, 2 * map_resp_.map.info.width);
      }

    protected:

      void computeCriticalRegions (double critical_epsilon) {

        // Compute critical points
        size_t pixel_critical_epsilon = 
          critical_epsilon / map_resp_.map.info.resolution;

        for (size_t i = 0; i < voronoi_points_.size(); ++i) {
          VoronoiPoint &vpi = voronoi_points_[i];
          float average_neighbourhood_clearance = 0;
          size_t neighbour_count = 0;
          bool is_clearance_minima = true;
          // Get all voronoi points in a region around this voronoi point
          for (size_t j = 0; j < voronoi_points_.size(); ++j) {
            // Don't check if it is the same point
            if (j == i) {
              continue;
            }

            // Compute distance of jth point to ith point - 
            // don't compare if too far away
            VoronoiPoint &vpj = voronoi_points_[j];
            float distance = 
              sqrt((vpj.x - vpi.x) * (vpj.x - vpi.x) + 
                  (vpj.y - vpi.y) * (vpj.y - vpi.y));
            if (distance > pixel_critical_epsilon) {
              continue;
            }

            average_neighbourhood_clearance += vpj.average_clearance;
            ++neighbour_count;

            // See if the jth point has lower clearance than the ith
            if (vpj.average_clearance < vpi.average_clearance) {
              is_clearance_minima = false;
              break;
            }
          }

          // If no neighbours, then this cannot be a critical point
          if (neighbour_count == 0) {
            continue;
          }

          // Check if the point is indeed better than the neighbourhood
          // This can happen is any 2 walls of the map are too straight
          average_neighbourhood_clearance /= neighbour_count;
          if (vpi.average_clearance >= average_neighbourhood_clearance) {
            continue;
          }
          vpi.critical_clearance_diff = 
            average_neighbourhood_clearance - vpi.average_clearance;

          std::vector<size_t> mark_for_removal;
          // This removal is not perfect, but ensures you don't have critical 
          // points too close.
          for (size_t j = 0; j < critical_points_.size(); ++j) {

            // Check if in same neighbourhood
            VoronoiPoint &vpj = critical_points_[j];
            float distance = 
              sqrt((vpj.x - vpi.x) * (vpj.x - vpi.x) + 
                  (vpj.y - vpi.y) * (vpj.y - vpi.y));
            if (distance > pixel_critical_epsilon) {
              continue;
            }

            // If in same neighbourhood, retain better point
            if (vpj.critical_clearance_diff >= vpi.critical_clearance_diff) {
              is_clearance_minima = false;
              break;
            } else {
              mark_for_removal.push_back(j);
            }
          }

          if (is_clearance_minima) {
            // Let's remove any points marked for removal
            for (size_t j = mark_for_removal.size() - 1; 
                j < mark_for_removal.size(); --j) { //unsigned
              critical_points_.erase(
                  critical_points_.begin() + mark_for_removal[j]);
            }

            // And then add this critical point
            critical_points_.push_back(vpi);
          }
        }

        // Once you have critical lines, produce connected regions (4-connected)
        // draw the critical lines on to a copy of the map so that we can find
        // connected regions
        cv::Mat component_map_color;
        drawMap(component_map_color);
        cvtColor(component_map_color, component_map_, CV_RGB2GRAY);
        drawCriticalLines(component_map_);

        component_number_.resize(
            component_map_.rows * component_map_.cols);
        ConnectedComponents cc(component_map_, component_number_);
        num_components_ = cc.getNumberComponents();

      }

      void drawCriticalLines(cv::Mat &image, 
          uint32_t orig_x = 0, uint32_t orig_y = 0) {

        for (size_t i = 0; i < critical_points_.size(); ++i) {
          VoronoiPoint &vp = critical_points_[i];
          if (vp.basis_points.size() != 2) {
            std::cerr << "ERROR: Found a critical point with more than 2 basis" 
                      << "points. This should not have happened" << std::endl;
          } else {
            Point2d &p1(vp.basis_points[0]);
            Point2d &p2(vp.basis_points[1]);
            cv::line(image, 
                cv::Point(orig_x + p1.x, 
                          orig_y + inflated_map_.info.height - 1 - p1.y),
                cv::Point(orig_x + p2.x, 
                          orig_y + inflated_map_.info.height - 1 - p2.y),
                cv::Scalar(0),
                1, 4); // draw a 4 connected line
          }
        }
      }

      std::vector<VoronoiPoint> critical_points_;
      cv::Mat component_map_;
      std::vector<int32_t> component_number_;
      std::vector<cv::Vec3b> component_colors_;
      size_t num_components_;

  }; /* TopologicalMapper */

} /* topological_mapper */

#endif /* end of include guard: TOPOLOGICAL_MAPPER_EA94RQRE */
