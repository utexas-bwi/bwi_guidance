/**
 * \file  topological_mapper.cpp
 * \brief Implementation for the topological mapper 
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
 * $ Id: 03/04/2013 03:07:20 PM piyushk $
 *
 **/

#include <topological_mapper/topological_mapper.h>
#include <topological_mapper/connected_components.h>
#include <topological_mapper/map_utils.h>
#include <topological_mapper/point_utils.h>

namespace topological_mapper {

  /**
   * \brief   computes the topological graph given the threshold for 
   *          VoronoiApproximator and a parameter controlling the size of 
   *          critical regions.
   * \param   threshold same as threhold in  VoronoiApproximator()
   * \param   critical_epsilon (meters) no 2 critical points can be closer
   *          than this distance.
   * \param   merge_threshold graph vertices having a smaller area than this
   *          should be merged together (meter^2)
   */
  void TopologicalMapper::computeTopologicalGraph(double threshold, 
      double critical_epsilon, double merge_threshold) {

    std::cout << "computeTopologicalGraph(): find voronoi points" << std::endl;
    findVoronoiPoints(threshold);
    std::cout << "computeTopologicalGraph(): compute critical regions" << std::endl;
    computeCriticalRegions(critical_epsilon);
    std::cout << "computeTopologicalGraph(): compute ze graph" << std::endl;
    computeGraph(merge_threshold);
  }

  /**
   * \brief   draws critical points and lines onto a given image starting at
   *          (orig_x, orig_y)
   */
  void TopologicalMapper::drawCriticalPoints(cv::Mat &image, 
      uint32_t orig_x, uint32_t orig_y) {

    for (size_t i = 0; i < critical_points_.size(); ++i) {
      VoronoiPoint &vp = critical_points_[i];
      cv::circle(image, 
          cv::Point(vp.x + orig_x, vp.y + orig_y), 2, cv::Scalar(0,0,255), -1);
    }

    drawCriticalLines(image, orig_x, orig_y);
  }

  /**
   * \brief   draws colored regions corresponding to each critical region
   *          onto a given image starting at (orig_x, orig_y)
   */
  void TopologicalMapper::drawConnectedComponents(cv::Mat &image,
      uint32_t orig_x, uint32_t orig_y) {

    // Figure out different colors - 1st color should always be black
    size_t num_colors = num_components_;
    component_colors_.resize(num_colors);
    for (size_t i = 0; i < num_colors; ++i) {
      component_colors_[i] = 
        cv::Vec3b(32 + rand() % 192, 32 + rand() % 192, 32 + rand() % 192);
    }

    // component 0 is obstacles + background. don't draw?

    // Now paint!
    for (uint32_t j = 1; j < map_resp_.map.info.height; ++j) {
      cv::Vec3b* image_row_j = image.ptr<cv::Vec3b>(j + orig_y);
      for (uint32_t i = 0; i < map_resp_.map.info.width; ++i) {
        size_t map_idx = MAP_IDX(map_resp_.map.info.width, i, j);
        if (component_map_[map_idx] == -1)
          continue;
        cv::Vec3b& pixel = image_row_j[i + orig_x];
        pixel[0] = component_colors_[component_map_[map_idx]][0];
        pixel[1] = component_colors_[component_map_[map_idx]][1];
        pixel[2] = component_colors_[component_map_[map_idx]][2];

      }
    }
  }


  void TopologicalMapper::drawRegionGraph(cv::Mat &image,
      uint32_t orig_x, uint32_t orig_y) {
    drawGraph(image, region_graph_, orig_x, orig_y);
  }

  void TopologicalMapper::drawPointGraph(cv::Mat &image,
      uint32_t orig_x, uint32_t orig_y) {
    drawGraph(image, point_graph_, orig_x, orig_y);
  }

  void TopologicalMapper::writeRegionGraphToFile(std::string &filename) {
    writeGraphToFile(filename, region_graph_, map_resp_.map.info);
  }

  void TopologicalMapper::writePointGraphToFile(std::string &filename) {
    writeGraphToFile(filename, point_graph_, map_resp_.map.info);
  }

  /**
   * \brief   Prints information about all the critical points to screen. 
   *          Used for testing the output of Topological Mapper.
   */
  void TopologicalMapper::printCriticalPoints() {
    for (size_t i = 0; i < critical_points_.size(); ++i) {
      VoronoiPoint &vp = critical_points_[i];
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
   * \brief   Draws the base map, voronoi points and critical points, lines
   *          and regions on to image. Should be only used for testing the 
   *          output for Topological Mapper.
   * \param   image OpenCV image we are drawing output on 
   */
  void TopologicalMapper::drawOutput(cv::Mat &image) {
    VoronoiApproximator::drawOutput(image);
    drawMap(image, 2 * map_resp_.map.info.width);
    drawConnectedComponents(image, 2 * map_resp_.map.info.width);
    drawCriticalPoints(image, 2 * map_resp_.map.info.width);
    drawMap(image, 3 * map_resp_.map.info.width);
    drawRegionGraph(image, 3 * map_resp_.map.info.width);
    drawMap(image, 4 * map_resp_.map.info.width);
    drawPointGraph(image, 4 * map_resp_.map.info.width);
  }

  /**
   * \brief   Compute critical regions once the voronoi points have been
   *          calculated
   * \param   critical_epsilon see TopologicalMapper() for more details
   */
  void TopologicalMapper::computeCriticalRegions (double critical_epsilon) {

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
        float distance = topological_mapper::getMagnitude(vpj - vpi); 
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
        float distance = norm(vpj - vpi); 
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

    // Remove any critical points where the point itself does not lie on the line
    std::vector<size_t> mark_for_removal;
    for (size_t i = 0; i < critical_points_.size(); ++i) {
      VoronoiPoint &cp = critical_points_[i];
      float theta0 = 
        atan2((cp.basis_points[0] - cp).y,
              (cp.basis_points[0] - cp).x);
      float theta1 = 
        atan2((cp.basis_points[1] - cp).y,
              (cp.basis_points[1] - cp).x);
      float thetadiff = fabs(theta1 - theta0);

      // We don't need to worry about wrapping here due known range of atan2
      if (thetadiff > M_PI + M_PI/12 || thetadiff < M_PI - M_PI/12) {
        mark_for_removal.push_back(i);
      }
    }

    // Remove bad critical points
    for (size_t j = mark_for_removal.size() - 1; 
        j < mark_for_removal.size(); --j) { //unsigned
      critical_points_.erase(
          critical_points_.begin() + mark_for_removal[j]);
    }

    // Once you have critical lines, produce connected regions (4-connected)
    // draw the critical lines on to a copy of the map so that we can find
    // connected regions
    cv::Mat component_map_color;
    drawMap(component_map_color, inflated_map_);
    cvtColor(component_map_color, component_image_, CV_RGB2GRAY);
    drawCriticalLines(component_image_);

    component_map_.resize(
        component_image_.rows * component_image_.cols);
    ConnectedComponents cc(component_image_, component_map_);
    num_components_ = cc.getNumberComponents();

  }

  void TopologicalMapper::computeGraph(double merge_threshold) {

    // First for each critical point, find out the neigbouring regions
    std::vector<std::set<uint32_t> > point_neighbour_sets;
    point_neighbour_sets.resize(critical_points_.size());

    // Draw the critical lines as their indexes on the map
    cv::Mat lines(map_resp_.map.info.height, map_resp_.map.info.width, CV_16UC1,
        cv::Scalar((uint16_t)-1));
    drawCriticalLines(lines, 0, 0, true);

    // Go over all the pixels in the lines image and find neighbouring critical
    // regions
    for (int j = 0; j < lines.rows; ++j) {
      uint16_t* image_row_j = lines.ptr<uint16_t>(j);
      for (int i = 0; i < lines.cols; ++i) {
        uint16_t pixel = image_row_j[i];
        if (pixel == (uint16_t)-1) {
          continue;
        }
        int x_offset[] = {0, -1, 1, 0};
        int y_offset[] = {-1, 0, 0, 1};
        size_t num_neighbours = 4;
        for (size_t count = 0; count < num_neighbours; ++count) {
          uint32_t x_n = i + x_offset[count];
          uint32_t y_n = j + y_offset[count];
          if (x_n >= (uint16_t) lines.cols || y_n >= (uint16_t) lines.rows) {
            continue;
          }
          size_t map_idx = MAP_IDX(lines.cols, x_n, y_n);
          if (component_map_[map_idx] >= 0 && 
              component_map_[map_idx] < (int32_t) num_components_) {
            point_neighbour_sets[pixel].insert(component_map_[map_idx]);
          }
        }
      }
    }

    std::vector<std::vector<uint32_t> > point_neighbours;
    point_neighbours.resize(critical_points_.size());
    for (size_t i = 0; i < critical_points_.size(); ++i) {
      if (point_neighbour_sets[i].size() == 2) {
        point_neighbours[i] = 
          std::vector<uint32_t>(point_neighbour_sets[i].begin(),
              point_neighbour_sets[i].end());
      } else {
        std::cout << "ERROR: one of the critical points has " << 
          point_neighbour_sets[i].size() << " neighbours. This means that " <<
          "the critical lines algorithm has failed.";
      }
    }

    // Find connectivity to remove any isolated sub-graphs
    std::vector<bool> checked_region(critical_points_.size(), false);
    std::vector<std::set<int> > graph_sets;
    for (size_t i = 0; i < num_components_; ++i) {
      if (checked_region[i]) {
        continue;
      }
      std::set<int> open_set, closed_set;
      open_set.insert(i);
      while (open_set.size() != 0) {
        int current_region = *(open_set.begin());
        open_set.erase(open_set.begin());
        closed_set.insert(current_region);
        checked_region[current_region] = true;
        for (int j = 0; j < critical_points_.size(); ++j) {
          for (int k = 0; k < 2; ++k) {
            if (point_neighbours[j][k] == current_region && 
                std::find(closed_set.begin(), closed_set.end(), 
                  point_neighbours[j][1-k]) == 
                closed_set.end()) {
              open_set.insert(point_neighbours[j][1-k]);
            }
          }
        }
      }
      graph_sets.push_back(closed_set);
    }

    std::set<int> master_region_set;
    if (graph_sets.size() != 1) {
      std::cout << "WARNING: Master graph is fragmented into " <<
        graph_sets.size() << " sub-graphs!!!" << std::endl;
      int max_idx = 0;
      int max_size = graph_sets[0].size();
      for (size_t i = 1; i < graph_sets.size(); ++i) {
        if (graph_sets[i].size() > max_size) {
          max_idx = i;
          max_size = graph_sets[i].size();
        }
      }
      master_region_set = graph_sets[max_idx];
    } else {
      master_region_set = graph_sets[0];
    }
    
    // Print neighbours
    // for (size_t i = 0; i < critical_points_.size(); ++i) {
    //   std::cout << i << " -> ";
    //   for (std::set<uint32_t>::iterator it = point_neighbours[i].begin();
    //       it != point_neighbours[i].end(); ++it) {
    //
    //     std::cout << *it << " ";
    //   }
    //   std::cout << std::endl;
    // }
    
    // Create the region graph next
    std::map<int, int> region_to_vertex_map;
    std::map<int, int> vertex_to_region_map;
    int vertex_count = 0;
    for (size_t r = 0; r < num_components_; ++r) { 
      if (std::find(master_region_set.begin(), master_region_set.end(), r) ==
          master_region_set.end()) {
        region_to_vertex_map[r] = -1;
        continue;
      }

      Graph::vertex_descriptor vi = boost::add_vertex(region_graph_);

      // Calculate the centroid
      uint32_t avg_i = 0, avg_j = 0, pixel_count = 0;
      for (size_t j = 0; j < map_resp_.map.info.height; ++j) {
        for (size_t i = 0; i < map_resp_.map.info.width; ++i) {
          size_t map_idx = MAP_IDX(map_resp_.map.info.width, i, j);
          if (component_map_[map_idx] == (int)r) {
            avg_j += j;
            avg_i += i;
            pixel_count++;
          }
        }
      }

      region_graph_[vi].location.x = ((float) avg_i) / pixel_count;
      region_graph_[vi].location.y = ((float) avg_j) / pixel_count;
      region_graph_[vi].pixels = floor(sqrt(pixel_count));
      std::cout << "mapping " << r << " to " << vertex_count << std::endl;
      region_to_vertex_map[r] = vertex_count;
      vertex_to_region_map[vertex_count] = r;
      ++vertex_count;
    }

    // Create 1 edge per critical point
    for (size_t i = 0; i < critical_points_.size(); ++i) {
      std::cout << "cp " << i << " adjacent to " << point_neighbours[i][0] <<
        " and " << point_neighbours[i][1] << std::endl;
      int region1 = region_to_vertex_map[point_neighbours[i][0]];
      int region2 = region_to_vertex_map[point_neighbours[i][1]];
      if (region1 == -1) {
        // part of one of the sub-graphs that was discarded
        continue;
      }
      int count = 0;
      Graph::vertex_descriptor vi,vj;
      vi = boost::vertex(region1, region_graph_);
      vj = boost::vertex(region2, region_graph_);
      Graph::edge_descriptor e; bool b;
      boost::tie(e,b) = boost::add_edge(vi, vj, region_graph_);
      region_graph_[e].weight = 
          topological_mapper::getMagnitude(region_graph_[vi].location - region_graph_[vj].location);
    }


    // Refine the region graph into the point graph:
    //  - remove any unnecessary vertices
    //  - resolve all large regions into critical point based vertices
    int pixel_threshold = merge_threshold / map_resp_.map.info.resolution;
    Graph::vertex_iterator vi, vend;
    vertex_count = 0;
    std::vector<int> critical_points_in_graph;
    for (boost::tie(vi, vend) = boost::vertices(region_graph_); vi != vend;
        ++vi, ++vertex_count) {
      
      // See if this only has 2 neighbours, and the 2 neighbours are visible to
      // each other
      Graph::adjacency_iterator ai, aend;
      std::vector<Point2f> adjacent_vertices;
      for (boost::tie(ai, aend) = boost::adjacent_vertices(
            (Graph::vertex_descriptor)*vi, region_graph_); 
          ai != aend; ++ai) {
        adjacent_vertices.push_back(region_graph_[*ai].location);
      }
      if (adjacent_vertices.size() == 2) {
        // Check if the 2 adjacent vertices are visible
        bool locations_visible = 
          topological_mapper::locationsInDirectLineOfSight(
            adjacent_vertices[0], adjacent_vertices[1], map_resp_.map);
        if (locations_visible) {
          continue;
        }
      }

      // See if the area of this region is too big to be in the point map
      if (region_graph_[*vi].pixels >= pixel_threshold) {
        // Get underlying region
        int region = vertex_to_region_map[vertex_count];
        for (size_t i = 0; i < critical_points_.size(); ++i) {
          if (point_neighbour_sets[i].count(region)) {
            critical_points_in_graph.push_back(i);
          }
        }
        continue;
      }

      // Otherwise insert this as is into the point graph
      Graph::vertex_descriptor point_vi = boost::add_vertex(point_graph_);
      point_graph_[point_vi] = region_graph_[*vi];
    }


     

    // Initialize the critical point arrays for forming the graph
    // std::vector<size_t> critical_pt_to_region_map;

    // for (size_t i = 0; i < critical_points_.size(); ++i) {
    //   critical_pt_to_region_map.push_back((size_t)-1);
    // }

    // // Reduce the point graph as necessary
    // size_t pixel_threshold = merge_threshold / map_resp_.map.info.resolution;
    // std::cout << "using pixel threshold: " << std::endl;

    // // Look for small regions
    // Graph::vertex_iterator vi, vend;
    // size_t region_count = 0;
    // for (boost::tie(vi, vend) = boost::vertices(region_graph_); vi != vend;
    //     ++vi, ++region_count) {

    //   // Check if this region interacts with more than 2 other regions
    //   Graph::adjacency_iterator ai, aend;
    //   size_t count = 0;
    //   for (boost::tie(ai, aend) = boost::adjacent_vertices(
    //         (Graph::vertex_descriptor)*vi, region_graph_); 
    //       ai != aend; ++ai) {
    //     count++;
    //   }

    //   std::cout << "Region " << region_count << " has " << count << " neighbours" << std::endl;
    //   for (size_t i = 0; i < critical_points_.size(); ++i) {
    //     if (point_neighbours[i].size() == 2) {
    //       if (point_neighbours[i].count(region_count)) {
    //         // This critical point needs to be mapped to this region
    //         std::cout << " contains critical pt " << i << std::endl;
    //       }
    //     }
    //   }

    //   if (count <= 1) {
    //     // TODO HACK that works since we don't have corridors that end in nothingness
    //     // This is a bad idea, but should work for now
    //     for (size_t i = 0; i < critical_points_.size(); ++i) {
    //       if (point_neighbours[i].size() == 2) {
    //         if (point_neighbours[i].count(region_count)) {
    //           // This critical point needs to be mapped to this region
    //           critical_pt_to_region_map[i] = (size_t) -2;
    //           std::cout << "dropping critical pt " << i << std::endl;
    //         }
    //       }
    //     }
    //   } else if (count > 2 && region_graph_[*vi].pixels < pixel_threshold) {
    //   // Now check whether all the critical points associated with this region
    //   // really close

    //   // This node needs to be added into the point graph instead of 
    //   // individual critical points
    //     for (size_t i = 0; i < critical_points_.size(); ++i) {
    //       if (point_neighbours[i].size() == 2) {
    //         if (point_neighbours[i].count(region_count)) {
    //           // This critical point needs to be mapped to this region
    //           critical_pt_to_region_map[i] = region_count;
    //           std::cout << "mapping critical pt " << i << " to region" << region_count << std::endl;
    //         }
    //       }
    //     }
    //   }
    // }

    // // Create the point graph first
    // std::vector<size_t> regions_added;
    // std::map<size_t, size_t> region_map;
    // std::map<size_t, size_t> critical_pt_map;
    // size_t vertex_count = 0;
    // for (size_t i = 0; i < critical_points_.size(); ++i) {
    //   if (critical_pt_to_region_map[i] == (size_t)-1) {
    //     Graph::vertex_descriptor vi = boost::add_vertex(point_graph_);
    //     point_graph_[vi].location.x = critical_points_[i].x;
    //     point_graph_[vi].location.y = critical_points_[i].y;
    //     point_graph_[vi].pixels = critical_points_[i].average_clearance;
    //     critical_pt_map[i] = vertex_count;
    //     std::cout << "mapping critical pt " << i << " to vertex " << vertex_count << std::endl;
    //     vertex_count++;
    //   } else if (critical_pt_to_region_map[i] != (size_t) -2) {
    //     if (std::find(regions_added.begin(), regions_added.end(), 
    //           critical_pt_to_region_map[i]) == regions_added.end()) {
    //       Graph::vertex_descriptor vi = boost::add_vertex(point_graph_);
    //       Graph::vertex_descriptor region = 
    //           boost::vertex(critical_pt_to_region_map[i], region_graph_);
    //       point_graph_[vi] = region_graph_[region];
    //       regions_added.push_back(critical_pt_to_region_map[i]);
    //       region_map[critical_pt_to_region_map[i]] = vertex_count;
    //       std::cout << "mapping critical pt " << i << " as region (first) " << 
    //         critical_pt_to_region_map[i] << " to vertex " << 
    //         vertex_count << std::endl;
    //       vertex_count++;
    //     } else {
    //       std::cout << "mapping critical pt " << i << " as region " << 
    //         critical_pt_to_region_map[i] << " to vertex " << 
    //         region_map[critical_pt_to_region_map[i]]  << std::endl;
    //     }
    //   }
    // }

    // // Construct the edges in this graph
    // for (size_t i = 0; i < critical_points_.size(); ++i) {
    //   if (point_neighbours[i].size() == 2) {
    //     for (std::set<uint32_t>::iterator it = point_neighbours[i].begin();
    //         it != point_neighbours[i].end(); ++it) {
    //       for (size_t j = i + 1; j < critical_points_.size(); ++j) {
    //         if (i == j)
    //           continue;
    //         if (point_neighbours[j].count(*it)) {
    //           Graph::vertex_descriptor vi,vj;
    //           if (critical_pt_to_region_map[i] == (size_t)-1) {
    //             vi = boost::vertex(critical_pt_map[i], point_graph_);
    //           } else {
    //             vi = boost::vertex(region_map[critical_pt_to_region_map[i]], point_graph_);
    //           }
    //           if (critical_pt_to_region_map[j] == (size_t)-1) {
    //             vj = boost::vertex(critical_pt_map[j], point_graph_);
    //           } else {
    //             vj = boost::vertex(region_map[critical_pt_to_region_map[j]], point_graph_);
    //           }
    //           if (vi != vj) {
    //             Graph::edge_descriptor e; bool b;
    //             boost::tie(e,b) = boost::add_edge(vi, vj, point_graph_);
    //             point_graph_[e].weight =
    //               topological_mapper::getMagnitude(point_graph_[vi].location - point_graph_[vj].location);
    //           }
    //         }
    //       }
    //     }
    //   }
    // }


  }

  /**
   * \brief   Draws critical lines (4-connected) onto given image starting
   *          at (orig_x, orig_y)
   */
  void TopologicalMapper::drawCriticalLines(cv::Mat &image, 
      uint32_t orig_x, uint32_t orig_y, bool draw_idx) {

    for (size_t i = 0; i < critical_points_.size(); ++i) {
      cv::Scalar color = cv::Scalar(0);
      if (draw_idx) {
        color = cv::Scalar((uint16_t)i);
      }
      VoronoiPoint &vp = critical_points_[i];
      if (vp.basis_points.size() != 2) {
        std::cerr << "ERROR: Found a critical point with more than 2 basis" 
          << "points. This should not have happened" << std::endl;
      } else {
        Point2d &p1(vp.basis_points[0]);
        Point2d &p2(vp.basis_points[1]);
        cv::line(image, 
            cv::Point(orig_x + p1.x, orig_y + p1.y),
            cv::Point(orig_x + p2.x, orig_y + p2.y), 
            color,
            1, 4); // draw a 4 connected line
      }
    }
  }

} /* topological_mapper */

