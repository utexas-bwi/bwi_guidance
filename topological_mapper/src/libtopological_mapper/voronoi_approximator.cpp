/**
 * \file  voronoi_approximator.cpp
 * \brief  Implementation for the voronoi approximator
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
 * $ Id: 03/04/2013 02:44:42 PM piyushk $
 *
 **/

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <topological_mapper/voronoi_approximator.h>

namespace topological_mapper {

  /**
   * \brief   Computes the points that lie on the Voronoi Graph using an 
   *          approximate strategy useful for voronoi approximation during
   *          mapping
   * \param   threshold minimum obstacle distance in meters Any point which 
   *          is less than threshold away from an obstacle is rejected as a
   *          voronoi candidate. This allows for not caring about minor
   *          breaks in a wall which can happen for any SLAM algorithm
   */
  void VoronoiApproximator::findVoronoiPoints(double threshold) {

    // Get the inflated cost map
    inflateMap(threshold, map_resp_.map, inflated_map_);

    // Compute the voronoi points
    double pixel_threshold = 
      ceil(threshold / map_resp_.map.info.resolution);

    uint32_t max_dimension = 
      std::max(inflated_map_.info.height, inflated_map_.info.width);

    for (uint32_t j = 0; j < inflated_map_.info.height; j++) {
      for (uint32_t i = 0; i < inflated_map_.info.width; i++) {

        // Check if this location is too close to a given obstacle
        uint32_t map_idx = MAP_IDX(inflated_map_.info.width, i, j);
        if (inflated_map_.data[map_idx] != 0) {
          continue;
        }

        VoronoiPoint vp;
        vp.x = i;
        vp.y = j;

        // Use the boxes to find obstacles
        for (uint32_t box = pixel_threshold; box < max_dimension; ++box) {

          // Early termination crit - if no hope of finding close obstacle
          if (vp.basis_points.size() != 0 &&
              box > vp.basis_distance + 1) {
            break;
          }

          // Get obstacles at this box size
          std::vector<Point2d> obstacles;
          uint32_t low_j = std::max(0, (int)j - (int)box);
          uint32_t high_j = 
            std::min(map_resp_.map.info.height - 1, j + box);
          uint32_t low_i = std::max(0, (int)i - (int)box);
          uint32_t high_i = std::min(map_resp_.map.info.width - 1, i + box);

          // Corners of the box + vertical edges
          for (uint32_t j_box = low_j; j_box <= high_j; ++j_box) {
            for (uint32_t i_box = low_i; i_box <= high_i; 
                i_box += high_i - low_i) {
              uint32_t map_idx_box = 
                MAP_IDX(inflated_map_.info.width, i_box, j_box);
              if (map_resp_.map.data[map_idx_box] != 0) {
                Point2d p;
                p.x = i_box;
                p.y = j_box;
                p.distance_from_ref = 
                  sqrt(
                      ((int32_t)j_box - j) * ((int32_t)j_box - j) +
                      ((int32_t)i_box - i) * ((int32_t)i_box - i)
                      );
                obstacles.push_back(p);
              }
            }
          }

          // horizontal edges
          for (uint32_t j_box = low_j; j_box <= high_j;
              j_box += high_j - low_j) {
            for (uint32_t i_box = low_i + 1; i_box < high_i + 1; ++i_box) {
              uint32_t map_idx_box = 
                MAP_IDX(inflated_map_.info.width, i_box, j_box);
              if (map_resp_.map.data[map_idx_box] != 0) {
                Point2d p;
                p.x = i_box;
                p.y = j_box;
                p.distance_from_ref = 
                  sqrt(
                      ((int32_t)j_box - j) * ((int32_t)j_box - j) +
                      ((int32_t)i_box - i) * ((int32_t)i_box - i)
                      );
                obstacles.push_back(p);
              }
            }
          }

          // Check if any obstacles are found
          if (obstacles.size() == 0) {
            continue;
          }

          // Now that obstacles are available, sort and add to the vp as
          // necessary
          std::sort(obstacles.begin(), obstacles.end(), 
              Point2dDistanceComp());

          for (size_t q = 0; q < obstacles.size(); q++) {
            if (vp.basis_points.size() == 0 ||
                obstacles[q].distance_from_ref <= vp.basis_distance + 1) {
              vp.addBasisCandidate(obstacles[q], pixel_threshold, 
                  inflated_map_);
            }
          }
        }

        if (vp.basis_points.size() >= 2) {
          voronoi_points_.push_back(vp);
        }
      }
    }

    // Compute average basis distance for each voronoi point
    for (size_t i = 0; i < voronoi_points_.size(); ++i) {
      VoronoiPoint &vp = voronoi_points_[i];
      float basis_distance_sum = 0;
      for (size_t j = 0; j < vp.basis_points.size(); ++j) {
        basis_distance_sum += vp.basis_points[j].distance_from_ref;
      }
      vp.average_clearance = basis_distance_sum / vp.basis_points.size();
    }

    // Label the voronoi diagram as being available
    initialized_ = true;
  }

  /**
   * \brief   Draws the base map and voronoi points on to image. Should be
   *          only used for testing the output for Voronoi Approximator.
   * \param   image OpenCV image we are drawing output on 
   */
  void VoronoiApproximator::drawOutput(cv::Mat &image) {
    if (!initialized_) {
      throw std::runtime_error("drawOutput(): voronoi diagram not "
          "initialized, call findVoronoiPoints first");
    }
    drawMap(image);
    drawMap(image, map_resp_.map.info.width);
    drawVoronoiPoints(image, map_resp_.map.info.width);
  }

  /**
   * \brief   Prints information about all the voronoi points to screen. 
   *          Used for testing the output for Voronoi Approximator.
   */
  void VoronoiApproximator::printVoronoiPoints() {
    for (size_t i = 0; i < voronoi_points_.size(); ++i) {
      VoronoiPoint &vp = voronoi_points_[i];
      std::cout << " (" << vp.x << "," << vp.y << "): ";
      for (size_t j = 0; j < vp.basis_points.size(); ++j) {
        std::cout << " (" << vp.basis_points[j].x << "," 
          << vp.basis_points[j].y << "," 
          << vp.basis_points[j].distance_from_ref << "), ";
      }
      std::cout << std::endl;
    }
  }

  /**
   * \brief   Draw voronoi points onto image starting at (orig_x, orig_y)
   * \param   image OpenCV image we are writing the map onto
   */
  void VoronoiApproximator::drawVoronoiPoints(cv::Mat &image, 
      uint32_t orig_x, uint32_t orig_y) {
    for (size_t i = 0; i < voronoi_points_.size(); ++i) {
      VoronoiPoint &vp = voronoi_points_[i];
      image.at<cv::Vec3b>
        (orig_y + inflated_map_.info.height - 1 - vp.y, vp.x + orig_x) = 
        cv::Vec3b(255,0,0);
    }
  }

} /* topological_mapper */
