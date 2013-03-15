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

#include <boost/lexical_cast.hpp>

#include <topological_mapper/topological_mapper.h>
#include <topological_mapper/connected_components.h>

namespace topological_mapper {

  /**
   * \brief   computes the topological graph given the threshold for 
   *          VoronoiApproximator and a parameter controlling the size of 
   *          critical regions.
   * \param   threshold same as threhold in  VoronoiApproximator()
   * \param   critical_epsilon (meters) no 2 critical points can be closer
   *          than this distance.
   */
  void TopologicalMapper::computeTopologicalGraph(double threshold, 
      double critical_epsilon) {

    findVoronoiPoints(threshold);
    computeCriticalRegions(critical_epsilon);
    computeGraph();
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

  void TopologicalMapper::drawGraph(cv::Mat &image, const Graph& graph,
      uint32_t orig_x, uint32_t orig_y) {

    Graph::vertex_iterator vi, vend;
    size_t count = 0;
    for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {

      // Draw this vertex
      Point2f location = graph[*vi].location;
      size_t vertex_size = 3; // + graph[*vi].pixels / 10;
      cv::Point vertex_loc(orig_x + (uint32_t)location.x, 
            orig_y + (uint32_t)location.y);
      cv::Point text_loc = vertex_loc + cv::Point(4,4);
      cv::circle(image, vertex_loc, vertex_size, cv::Scalar(0,0,255), -1);
      cv::putText(image, boost::lexical_cast<std::string>(count), text_loc,
        cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(0,0,255), 1, CV_AA);

      // Draw the edges from this vertex
      Graph::adjacency_iterator ai, aend;
      for (boost::tie(ai, aend) = boost::adjacent_vertices(
            (Graph::vertex_descriptor)*vi, graph.graph()); 
          ai != aend; ++ai) {
        Point2f location2 = graph[*ai].location;
        cv::line(image, 
            cv::Point(orig_x + location.x, orig_y + location.y),
            cv::Point(orig_x + location2.x, orig_y + location2.y),
            cv::Scalar(0, 0, 255),
            1, 4); // draw a 4 connected line
      }

      count++;
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

  void TopologicalMapper::writeGraphToFile(std::string &filename, 
      const Graph& graph) {

    std::map<Graph::vertex_iterator, size_t> vertex_map;
    size_t count = 0;
    for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
      vertex_map[vi] = count;
      count++;
    }

    count = 0;
    for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {






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

    // Remove any critical points where the point itself does not lie on the line
    std::vector<size_t> mark_for_removal;
    for (size_t i = 0; i < critical_points_.size(); ++i) {
      VoronoiPoint &cp = critical_points_[i];
      float theta0 = 
        atan2((int32_t)cp.basis_points[0].y - (int32_t)cp.y, 
              (int32_t)cp.basis_points[0].x - (int32_t)cp.x);
      float theta1 = 
        atan2((int32_t)cp.basis_points[1].y - (int32_t)cp.y, 
              (int32_t)cp.basis_points[1].x - (int32_t)cp.x);
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

  void TopologicalMapper::computeGraph() {

    // First for each critical point, find out the neigbouring regions
    std::vector< std::set<uint32_t> > point_neighbours;
    point_neighbours.resize(critical_points_.size());

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
            point_neighbours[pixel].insert(component_map_[map_idx]);
          }
        }
      }
    }

    // Print neighbours
    // for (size_t i = 0; i < critical_points_.size(); ++i) {
    //   std::cout << i << " -> ";
    //   for (std::set<uint32_t>::iterator it = point_neighbours[i].begin();
    //       it != point_neighbours[i].end(); ++it) {
    //     std::cout << *it << " ";
    //   }
    //   std::cout << std::endl;
    // }
    
    // Create the point graph first
    for (size_t i = 0; i < critical_points_.size(); ++i) {
      boost::add_vertex(i, point_graph_);
      point_graph_[i].location.x = critical_points_[i].x;
      point_graph_[i].location.y = critical_points_[i].y;
      point_graph_[i].pixels = critical_points_[i].average_clearance;
    }
    // Construct the edges in this graph
    for (size_t i = 0; i < critical_points_.size(); ++i) {
      if (point_neighbours[i].size() == 2) {
        for (std::set<uint32_t>::iterator it = point_neighbours[i].begin();
            it != point_neighbours[i].end(); ++it) {
          for (size_t j = 0; j < critical_points_.size(); ++j) {
            if (i == j)
              continue;
            if (point_neighbours[j].count(*it)) {
              boost::add_edge_by_label(i, j, point_graph_);
            }
          }
        }
      }
    }

    // Create the region graph next
    for (size_t r = 0; r < num_components_; ++r) { 
      boost::add_vertex(r, region_graph_);

      // Calculate the centroid
      uint32_t avg_i = 0, avg_j = 0, count = 0;
      for (size_t j = 0; j < map_resp_.map.info.height; ++j) {
        for (size_t i = 0; i < map_resp_.map.info.width; ++i) {
          size_t map_idx = MAP_IDX(map_resp_.map.info.width, i, j);
          if (component_map_[map_idx] == (int)r) {
            avg_j += j;
            avg_i += i;
            count++;
          }
        }
      }

      region_graph_[r].location.x = ((float) avg_i) / count;
      region_graph_[r].location.y = ((float) avg_j) / count;
      region_graph_[r].pixels = floor(sqrt(count)/2);
    }
    // Create 1 edge per critical point
    for (size_t i = 0; i < critical_points_.size(); ++i) {
      if (point_neighbours[i].size() == 2) {
        int count = 0;
        uint32_t v[] = {0, 0};
        for (std::set<uint32_t>::iterator it = point_neighbours[i].begin();
            it != point_neighbours[i].end(); ++it, ++count) {
          v[count] = *it;
        }
        boost::add_edge_by_label(v[0], v[1], region_graph_);
      }
    }

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

