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
#include <cstdlib>

#include <topological_mapper/map_loader.h>
#include <topological_mapper/map_inflator.h>
#include <topological_mapper/point.h>

namespace topological_mapper {

  class ConnectedComponents {
    /* data */
  public:
    ConnectedComponents (const nav_msgs::OccupancyGrid& map, std::vector<int32_t>& component_map) :
        component_number_(component_map), map_(map) {

      // Label all non-free spaces as obstacles in the component space
      for (size_t i = 0; i < map_.data.size(); ++i) {
        if (map_.data[i] != 0) {
          component_number_[i] = 0;
        }
      }
      current_component_number_ = 1;

      // Run simple connected components to figure out component labellings and centroids
      for (size_t j = 0; j < map_.info.height; ++j) {
        for (size_t i = 0; i < map_.info.width; ++i) {
          uint32_t map_idx = MAP_IDX(map_.info.width, i, j);
          // Check if location has already been labelled
          if (component_number_[map_idx] != -1) {
            continue;
          }
          labelFrom(i, j);
          ++current_component_number_;
        }
      }
    }
    std::vector<int32_t> &component_number_;
    size_t current_component_number_;

  private:

    void labelFrom(size_t x, size_t y) {
      // Label this idx
      uint32_t map_idx = MAP_IDX(map_.info.width, x, y);
      component_number_[map_idx] = current_component_number_;

      // Check for unlabelled neighbours and mark them as well
      size_t neighbour_count = 4;
      int32_t x_offset[] = {0, -1, 1, 0};
      int32_t y_offset[] = {-1, 0, 0, 1};
      for (size_t i = 0; i < neighbour_count; ++i) {
        // Check if neighbours are still on map
        Point2d p;
        p.x = (int)x + x_offset[i];
        p.y = (int)y + y_offset[i];
        //std::cout << " " << p.x << " " << p.y << std::endl;
        if (p.x >= map_.info.width || p.y >= map_.info.height) { //covers negative case as well (unsigned)
          continue;
        }
        uint32_t map_idx = MAP_IDX(map_.info.width, p.x, p.y);
        if (component_number_[map_idx] != -1) {
          // Not a free vertex
          continue;
        }
        //std::cout << x << " " << y << " -> " << p.x << " " << p.y << std::endl;

        labelFrom(p.x, p.y);
      }
    }

    const nav_msgs::OccupancyGrid& map_;
  };


  class VoronoiPoint {
    public:
      Point2d point;
      std::vector<Point2d> basis_points;

      float basis_distance; // minimum clearance to a basis point - used while computing basis points
      float average_clearance; // average clearance from all basis points - used after all basis points have been computed
      float critical_clearance_diff; // if this point is a critical point, how lower is the clearance of this point in comparison of its neighbours
      
      void addBasisCandidate(const Point2d& candidate, uint32_t threshold, 
          const nav_msgs::OccupancyGrid& map) {

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

        // Get the directed DFS searcher
        DirectedDFS dfs(map);

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

            // See if these basis points are really close by searching for a
            // short path in the walls
            if (dfs.searchForPath(basis_points[i], basis_points[j], 2 * basis_distance)) {
              elements_to_erase.push_back(i);
              break;
            }

          }
        }

        // Actually remove elements from the basis point array
        for (size_t i = elements_to_erase.size() - 1; i <= elements_to_erase.size(); --i) {
          basis_points.erase(basis_points.begin() + elements_to_erase[i]);
        }
        
      }

  };

  class CriticalPoint : public VoronoiPoint {
    public:
      size_t critical_region_1_idx_;
      size_t critical_region_2_idx_;

  }; /* CriticalPoint */

  class CriticalRegion {
    public:
      geometry_msgs::Point32
  }; /* CriticalRegion */

  class TopologicalGraph {
    public:


  };

  class VoronoiApproximator : public MapLoader {

    public:
      VoronoiApproximator(const std::string& fname) :
        MapLoader(fname), initialized_(false) { }

      void findVoronoiPoints(double threshold, double critical_epsilon) {

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
              uint32_t low_j = std::max(0, (int)j - (int)box);
              uint32_t high_j = std::min(map_resp_.map.info.height - 1, j + box);
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
                    p.distance_from_ref = sqrt(((int32_t)j_box - j) * ((int32_t)j_box - j) +
                        ((int32_t)i_box - i) * ((int32_t)i_box - i));
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
                  vp.addBasisCandidate(obstacles[q], pixel_threshold, inflated_map_);
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
          //std::cout << "vp: (" << vp.point.x << "," << vp.point.y << ") -> " << vp.average_clearance << std::endl;
        }

        // Compute critical points
        float pixel_critical_epsilon = critical_epsilon / map_resp_.map.info.resolution;
        //std::cout << "Critical epsilon (pxl): " << pixel_critical_epsilon << std::endl;
        for (size_t i = 0; i < voronoi_points_.size(); ++i) {
          VoronoiPoint &vpi = voronoi_points_[i];
          float average_neighbourhood_clearance = 0;
          size_t neighbour_count = 0;
          //std::cout << "vpi: (" << vpi.point.x << "," << vpi.point.y << ") -> " << vpi.average_clearance << std::endl;
          bool is_clearance_minima = true;
          // Get all voronoi points in a region around this voronoi point
          for (size_t j = 0; j < voronoi_points_.size(); ++j) {
            // Don't check if it is the same point
            if (j == i) {
              continue;
            }

            // Compute distance of jth point to ith point - don't compare if too far away
            VoronoiPoint &vpj = voronoi_points_[j];
            float distance = 
              sqrt((vpj.point.x - vpi.point.x) * (vpj.point.x - vpi.point.x) + 
                  (vpj.point.y - vpi.point.y) * (vpj.point.y - vpi.point.y));
            if (distance > pixel_critical_epsilon) {
              continue;
            }

            average_neighbourhood_clearance += vpj.average_clearance;
            ++neighbour_count;

            //std::cout << "vpj: (" << vpj.point.x << "," << vpj.point.y << ") -> " << vpj.average_clearance << std::endl;
            // See if the jth point has lower clearance than the ith
            if (vpj.average_clearance < vpi.average_clearance) {
              is_clearance_minima = false;
              break;
            }
          }

          // If no neighbours, then this cannot be a critical point
          if (neighbour_count == 0) {
            //continue;
          }

          // Check if the point is indeed better than the neighbourhood
          // This can happen is any 2 walls of the map are too straight
          average_neighbourhood_clearance /= neighbour_count;
          if (vpi.average_clearance >= average_neighbourhood_clearance) {
            //std::cout << vpi.average_clearance << " " << average_neighbourhood_clearance << std::endl;
            continue;
          }
          vpi.critical_clearance_diff = average_neighbourhood_clearance - vpi.average_clearance;

          std::vector<size_t> mark_for_removal;
          // This removal is not perfect, but ensures you don't have critical points too close.
          for (size_t j = 0; j < critical_points_.size(); ++j) {
            
            // Check if in same neighbourhood
            VoronoiPoint &vpj = critical_points_[j];
            float distance = 
              sqrt((vpj.point.x - vpi.point.x) * (vpj.point.x - vpi.point.x) + 
                  (vpj.point.y - vpi.point.y) * (vpj.point.y - vpi.point.y));
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
            for (size_t j = mark_for_removal.size() - 1; j < mark_for_removal.size(); --j) { //unsigned
              critical_points_.erase(critical_points_.begin() + mark_for_removal[j]);
            }

            // And then add this critical point
            critical_points_.push_back(vpi);
          }
        }

        // Once you have critical lines, produce connected regions (4-connected)
        // draw the critical lines on to a copy of the map
        component_map_.info = map_resp_.map.info;
        component_map_.data = map_resp_.map.data;
        drawCriticalLines(component_map_);

        component_number_.resize(component_map_.info.height * component_map_.info.width);
        for (size_t i = 0; i < component_number_.size(); ++i) {
          component_number_[i] = -1;
        }
        ConnectedComponents cc(component_map_, component_number_);
        num_components_ = cc.current_component_number_;

        // Finally, produce the topological graph
        // for each critical point, pick a basis point and see which 2 regions lie next



        // Label the voronoi diagram as being available
        initialized_ = true;
      }

      void drawOutput(cv::Mat &image) {
        if (!initialized_) {
          throw std::runtime_error("drawOutput(): voronoi diagram not "
              "initialized, call findVoronoiPoints first");
        }
        drawMap(image);
        //drawMap(image, inflated_map_, map_resp_.map.info.width); 
        drawMap(image, map_resp_.map.info.width);
        drawVoronoiPoints(image, map_resp_.map.info.width);
        drawMap(image, 2 * map_resp_.map.info.width);
        drawConnectedComponents(image, 2 * map_resp_.map.info.width);
        drawCriticalPoints(image, 2 * map_resp_.map.info.width);
        //printVoronoiPoints();
        //printCriticalPoints();
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

      void drawCriticalPoints(cv::Mat &image, 
          uint32_t orig_x = 0, uint32_t orig_y = 0) {
        for (size_t i = 0; i < critical_points_.size(); ++i) {
          VoronoiPoint &vp = critical_points_[i];
          //std::cout << "in" << orig_y + inflated_map_.info.height - 1 - vp.point.y << " " << vp.point.x + orig_x << std::endl;
          cv::circle(image, cv::Point(vp.point.x + orig_x, orig_y + inflated_map_.info.height - 1 - vp.point.y), 2, cv::Scalar(0,0,255), -1);
          // critical points should only have 2 basis points
          if (vp.basis_points.size() != 2) {
            std::cerr << "ERROR: Found a critical point with more than 2 basis points. This should not have happened" << std::endl;
          } else {
            Point2d &p1(vp.basis_points[0]);
            Point2d &p2(vp.basis_points[1]);
            cv::line(image, 
                cv::Point(p1.x + orig_x, orig_y + inflated_map_.info.height - 1 - p1.y),
                cv::Point(p2.x + orig_x, orig_y + inflated_map_.info.height - 1 - p2.y),
                cv::Scalar(0,0,255),
                1);
          }
        }
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

        // Now paint!
        for (uint32_t j = 0; j < map_resp_.map.info.height; ++j) {
          cv::Vec3b* image_row_j = image.ptr<cv::Vec3b>(j + orig_y);
          for (uint32_t i = 0; i < map_resp_.map.info.width; ++i) {
            size_t map_idx = MAP_IDX(map_resp_.map.info.width, i, map_resp_.map.info.height - j - 1);
            cv::Vec3b& pixel = image_row_j[i + orig_x];
            pixel[0] = component_colors_[component_number_[map_idx]][0];
            pixel[1] = component_colors_[component_number_[map_idx]][1];
            pixel[2] = component_colors_[component_number_[map_idx]][2];
            
          }
        }
      }

      void drawCriticalLines(nav_msgs::OccupancyGrid& map) {
        cv::Mat image(map.info.height, map.info.width, CV_8UC1, cv::Scalar(0));
        for (size_t i = 0; i < critical_points_.size(); ++i) {
          VoronoiPoint &vp = critical_points_[i];
          //std::cout << "in" << orig_y + inflated_map_.info.height - 1 - vp.point.y << " " << vp.point.x + orig_x << std::endl;
          if (vp.basis_points.size() != 2) {
            std::cerr << "ERROR: Found a critical point with more than 2 basis points. This should not have happened" << std::endl;
          } else {
            Point2d &p1(vp.basis_points[0]);
            Point2d &p2(vp.basis_points[1]);
            cv::line(image, 
                cv::Point(p1.x, inflated_map_.info.height - 1 - p1.y),
                cv::Point(p2.x, inflated_map_.info.height - 1 - p2.y),
                cv::Scalar(100),
                1);
          }
        }

        for (uint32_t j = 0; j < map.info.height; ++j) {
          uint8_t* image_row_j = image.ptr<uint8_t>(j);
          for (uint32_t i = 0; i < map.info.width; ++i) {
            size_t map_idx = MAP_IDX(map.info.width, i, map.info.height - 1 - j);
            uint8_t pixel = image_row_j[i];
            if (pixel != 0) {
              map.data[map_idx] = pixel;
            }
          }
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

      void printCriticalPoints() {
        for (size_t i = 0; i < critical_points_.size(); ++i) {
          VoronoiPoint &vp = critical_points_[i];
          std::cout << " (" << vp.point.x << "," << vp.point.y << "): ";
          for (size_t j = 0; j < vp.basis_points.size(); ++j) {
            std::cout << " (" << vp.basis_points[j].x << "," << vp.basis_points[j].y << "," << vp.basis_points[j].distance_from_ref << "), ";
          }
          std::cout << std::endl;
        }
      }

      std::vector<VoronoiPoint> voronoi_points_;
      std::vector<VoronoiPoint> critical_points_;
      nav_msgs::OccupancyGrid inflated_map_;
      nav_msgs::OccupancyGrid component_map_;
      std::vector<int32_t> component_number_;
      std::vector<cv::Vec3b> component_colors_;
      size_t num_components_;
      bool initialized_;
        
  }; /* VoronoiApproximator */
  
} /* topological_mapper */

#endif /* end of include guard: VORONOI_APPROXIMATOR_IVSRUILH */
