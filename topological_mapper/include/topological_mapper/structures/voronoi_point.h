/**
 * \file  voronoi_point.h
 * \brief  Base class for a voronoi point. Simple wrapper around Point2d that
 *         maintains a given separation between basis points
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
 * $ Id: 03/01/2013 01:17:50 PM piyushk $
 *
 **/

#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <topological_mapper/structures/point.h>

namespace topological_mapper {

  class VoronoiPoint : public Point2d {

    public:
      std::vector<Point2d> basis_points;

      /** /brief minimum clearance to a basis point - used while computing 
       *         basis points
       */
      float basis_distance;

      /** /brief average clearance from all basis points - 
       *         used after all basis points have been computed 
       */ 
      float average_clearance; 

      /** /brief if this point is a critical point, how lower is the clearance 
       *         of this point in comparison of its neighbours 
       */
      float critical_clearance_diff; 
      
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
        std::sort(basis_points.begin(), basis_points.end(), 
            point2dDistanceComp);
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
          std::vector<size_t>::iterator erase_iterator = 
            elements_to_erase.begin();

          for (size_t j = 0; j < i; ++j) {

            while (erase_iterator != elements_to_erase.end() && 
                *erase_iterator < j) {
              erase_iterator++;
            }

            if (erase_iterator != elements_to_erase.end() && 
                *erase_iterator == j) { 
              continue;
            }

            // See if basis point i is too close to basis point j. retain j
            uint32_t xj = basis_points[j].x;
            uint32_t yj = basis_points[j].y;
            float distance = sqrt((xi - xj)*(xi - xj) + (yi - yj) *(yi - yj));
            if (distance < 2 * threshold) {
              elements_to_erase.push_back(i); // does not affect erase_iterator
              break;
            }

            // See if these basis points are really close by searching for a
            // short path in the walls
            if (dfs.searchForPath(basis_points[i], basis_points[j], 
                2 * basis_distance)) {
              elements_to_erase.push_back(i);
              break;
            }
          }
        }

        // Actually remove elements from the basis point array
        for (size_t i = elements_to_erase.size() - 1; 
            i <= elements_to_erase.size(); --i) {
          basis_points.erase(basis_points.begin() + elements_to_erase[i]);
        }
      }

  }; /* VoronoiPoint */
  
} /* topological_mapper */
