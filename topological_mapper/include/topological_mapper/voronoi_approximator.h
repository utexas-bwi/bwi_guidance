/**
 * \file  voronoi_approximator.h
 * \brief  Constructs a voronoi approximation given a map of the world
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
 * $ Id: 02/21/2013 11:52:37 AM piyushk $
 *
 **/

#ifndef VORONOI_APPROXIMATOR_IVSRUILH
#define VORONOI_APPROXIMATOR_IVSRUILH

#include <algorithm>
#include <cmath>

#include <topological_mapper/map_loader.h>
#include <topological_mapper/map_inflator.h>

namespace topological_mapper {

  struct Point2d {
    uint32_t x;
    uint32_t y;
    float distance_from_ref;
  };

  struct Point2dDistanceComp {
    bool operator() (Point2d i, Point2d j) {
      return i.distance_from_ref < j.distance_from_ref;
    }
  } point2dDistanceComp;

  class VoronoiPoint {
    public:
      Point2d point;
      std::vector<Point2d> basis_points;

      float basis_distance;
      
      void addBasisCandidate(const Point2d& candidate, uint32_t threshold) {
        if (basis_points.size() == 0) {
          basis_points.push_back(candidate);
          basis_distance = candidate.distance_from_ref;
          return;
        }
        if (candidate.distance_from_ref > basis_distance + 1) {
          return;
        }

        basis_points.push_back(candidate);
        std::sort(basis_points.begin(), basis_points.end(), point2dDistanceComp);
        basis_distance = basis_points[0].distance_from_ref;

        // Mark elements that are too close to be erased
        std::vector<size_t> elements_to_erase;
        for (size_t i = 0; i < basis_points.size(); ++i) {

          // Check if the clearance for this point is much further away from the 
          // minimum clearance
          if (basis_points[i].distance_from_ref > basis_distance + 1) {
            elements_to_erase.push_back(i);
            break;
          }

          uint32_t xi = basis_points[i].x;
          uint32_t yi = basis_points[i].y;
          std::vector<size_t>::iterator erase_iterator = elements_to_erase.begin();
          for (size_t j = 0; j < i; ++j) {
            while (erase_iterator != elements_to_erase.end() && *erase_iterator < j)
              erase_iterator++;
            if (erase_iterator != elements_to_erase.end() && *erase_iterator == j) 
              continue;

            // See if basis point i is too close to basis point j. retain j
            uint32_t xj = basis_points[j].x;
            uint32_t yj = basis_points[j].y;
            float distance = sqrt((xi - xj)*(xi - xj) + (yi - yj) *(yi - yj));
            if (distance < 2 * threshold) {
              elements_to_erase.push_back(i); // should not affect erase_iterator
              break;
            }

            // See if this point is too close by 2nd temporary metric
            // if (distance < basis_distance) {
            //   elements_to_erase.push_back(i);
            //   break;
            // }
          }
        }

        // Actually remove elements from the basis point array
        for (size_t i = elements_to_erase.size() - 1; i <= elements_to_erase.size(); --i) {
          basis_points.erase(basis_points.begin() + elements_to_erase[i]);
        }
        
      }

  };

  class VoronoiApproximator : public MapLoader {

    public:
      VoronoiApproximator(const std::string& fname) :
        MapLoader(fname), initialized_(false) { }

      void findVoronoiPoints(double threshold) {

        // Get the inflated cost map
        inflateMap(threshold, map_resp_.map, inflated_map_);

        // Compute the voronoi points
        double pixel_threshold = 
          ceil(threshold / map_resp_.map.info.resolution);

        uint32_t max_dimension = 
          std::max(inflated_map_.info.height, inflated_map_.info.width);

        for (uint32_t j = 0; j < inflated_map_.info.height; j++) {
          std::cout << j << std::endl;
          for (uint32_t i = 0; i < inflated_map_.info.width; i++) {

            // Check if this location is too close to a given obstacle
            uint32_t map_idx = MAP_IDX(inflated_map_.info.width, i, j);
            if (inflated_map_.data[map_idx] != 0) {
              continue;
            }
            VoronoiPoint vp;
            vp.point.x = i;
            vp.point.y = j;

            // Use the boxes to find obstacles
            for (uint32_t box = pixel_threshold; box < max_dimension; ++box) {

              // Early termination crit - if no hope of finding close obstacle
              if (vp.basis_points.size() != 0 &&
                  box > vp.basis_distance + 1) {
                break;
              }

              // Get obstacles at this box size
              std::vector<Point2d> obstacles;
              uint32_t low_j = std::max((uint32_t)0, j - box);
              uint32_t high_j = std::min(map_resp_.map.info.height - 1, j + box);
              uint32_t low_i = std::max((uint32_t)0, i - box);
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
                    p.distance_from_ref = sqrt(((int32_t)j_box - j) * ((int32_t)j_box - j) +
                        ((int32_t)i_box - i) * ((int32_t)i_box - i));
                    // if (obstacles[q].distance_from_ref > 20) {
                    //   std::cout << obstacles[q].distance_from_ref << std::endl;
                    // }
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
                    p.distance_from_ref = sqrt(((int32_t)j_box - j) * ((int32_t)j_box - j) +
                        ((int32_t)i_box - i) * ((int32_t)i_box - i));
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
              std::sort(obstacles.begin(), obstacles.end(), point2dDistanceComp);
              for (size_t q = 0; q < obstacles.size(); q++) {
                if (vp.basis_points.size() == 0 ||
                    obstacles[q].distance_from_ref <= vp.basis_distance + 1) {
                  vp.addBasisCandidate(obstacles[q], pixel_threshold);
                }
              }
            }

            if (vp.basis_points.size() >= 2) {
              voronoi_points_.push_back(vp);
            }
          }
        }

        // Label the voronoi diagram as being available
        initialized_ = true;
      }

      void drawOutput(cv::Mat &image) {
        if (!initialized_) {
          throw std::runtime_error("drawOutput(): voronoi diagram not "
              "initialized, call findVoronoiPoints first");
        }
        drawMap(image);
        drawMap(image, inflated_map_, map_resp_.map.info.width); 
        drawMap(image, 2 * map_resp_.map.info.width);
        drawVoronoiPoints(image, 2 * map_resp_.map.info.width);
        //printVoronoiPoints();
      }


    protected:

      void drawVoronoiPoints(cv::Mat &image, 
          uint32_t orig_x = 0, uint32_t orig_y = 0) {
        for (size_t i = 0; i < voronoi_points_.size(); ++i) {
          VoronoiPoint &vp = voronoi_points_[i];
          image.at<cv::Vec3b>(orig_y + inflated_map_.info.height - 1 - vp.point.y, vp.point.x + orig_x) = 
            cv::Vec3b(255,0,0);
        }
      }

      void printVoronoiPoints() {
        for (size_t i = 0; i < voronoi_points_.size(); ++i) {
          VoronoiPoint &vp = voronoi_points_[i];
          std::cout << " (" << vp.point.x << "," << vp.point.y << "): ";
          for (size_t j = 0; j < vp.basis_points.size(); ++j) {
            std::cout << " (" << vp.basis_points[j].x << "," << vp.basis_points[j].y << "," << vp.basis_points[j].distance_from_ref << "), ";
          }
          std::cout << std::endl;
        }
      }

      std::vector<VoronoiPoint> voronoi_points_;
      nav_msgs::OccupancyGrid inflated_map_;
      bool initialized_;
        
  }; /* VoronoiApproximator */
  
} /* topological_mapper */

#endif /* end of include guard: VORONOI_APPROXIMATOR_IVSRUILH */
