/**
 * \file  connected_components.h
 * \brief  Connected Components implementation to get all the points in a
 *         critical region, along with neighbouring critical points
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
 * $ Id: 03/01/2013 01:04:40 PM piyushk $
 *
 **/

#ifndef CONNECTED_COMPONENTS_DVJRQLHV
#define CONNECTED_COMPONENTS_DVJRQLHV

namespace topological_mapper {

  class ConnectedComponents {

    public:
      ConnectedComponents (const nav_msgs::OccupancyGrid& map, 
          std::vector<int32_t>& component_map) :
        map_(map), component_map(component_map) {

          // Label all non-free spaces as obstacles in the component space
          for (size_t i = 0; i < map_.data.size(); ++i) {
            if (map_.data[i] != 0) {
              component_map[i] = 0;
            }
          }
          current_component_number_ = 1;

          // Run simple connected components to figure out component labellings 
          // and centroids
          for (size_t j = 0; j < map_.info.height; ++j) {
            for (size_t i = 0; i < map_.info.width; ++i) {
              uint32_t map_idx = MAP_IDX(map_.info.width, i, j);
              // Check if location has already been labelled
              if (component_map[map_idx] != -1) {
                continue;
              }
              labelFrom(i, j);
              ++current_component_number_;
            }
          }
        }

      size_t getNumberComponents() {
        return current_component_number_;
      }

    private:

      void labelFrom(size_t x, size_t y) {
        // Label this idx
        uint32_t map_idx = MAP_IDX(map_.info.width, x, y);
        component_map[map_idx] = current_component_number_;

        // Check for unlabelled neighbours and mark them as well
        size_t neighbour_count = 4;
        int32_t x_offset[] = {0, -1, 1, 0};
        int32_t y_offset[] = {-1, 0, 0, 1};
        for (size_t i = 0; i < neighbour_count; ++i) {
          // Check if neighbours are still on map
          Point2d p;
          p.x = (int)x + x_offset[i];
          p.y = (int)y + y_offset[i];
          //covers negative case as well (unsigned)
          if (p.x >= map_.info.width || p.y >= map_.info.height) { 
            continue;
          }
          uint32_t map_idx = MAP_IDX(map_.info.width, p.x, p.y);
          if (component_map[map_idx] != -1) {
            // Not a free vertex
            continue;
          }

          labelFrom(p.x, p.y);
        }
      }

      const nav_msgs::OccupancyGrid& map_;
      std::vector<int32_t> &component_map;
      size_t current_component_number_;

  }; /* ConnectedComponents */

} /* topological_mapper */

#endif /* end of include guard: CONNECTED_COMPONENTS_DVJRQLHV */
