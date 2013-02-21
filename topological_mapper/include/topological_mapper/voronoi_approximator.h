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

#include <topological_mapper/map_loader.h>
#include <topological_mapper/map_inflator.h>

namespace topological_mapper {

  class VoronoiApproximator : public MapLoader {

    public:
      VoronoiApproximator(const std::string& fname) :
        MapLoader(fname), initialized_(false) { }

      void findVoronoiPoints(double threshold) {
        initialized_ = true;
        inflateMap(threshold, map_resp_.map, inflated_map_);
        double pixel_threshold = ceil(threshold / map_resp_.map.info.resolution);
      }

      void drawOutput(cv::Mat &image) {
        if (!initialized_) {
          throw std::runtime_error("drawOutput(): voronoi diagram not initialized, call findVoronoiPoints first");
        }
        drawMap(image);
        drawMap(image, inflated_map_, map_resp_.map.info.width); 
      }

    protected:

      nav_msgs::OccupancyGrid inflated_map_;
      bool initialized_;
        
  }; /* VoronoiApproximator */
  
} /* topological_mapper */

#endif /* end of include guard: VORONOI_APPROXIMATOR_IVSRUILH */
