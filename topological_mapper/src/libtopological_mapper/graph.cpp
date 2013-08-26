/**
 * \file  graph.cpp
 * \brief  Implementation for graph functions
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
 * $ Id: 04/18/2013 05:10:28 PM piyushk $
 *
 **/

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <topological_mapper/graph.h>
#include <topological_mapper/map_utils.h>
#include <topological_mapper/point_utils.h>

namespace topological_mapper {

  /**
   * \brief   draws the given graph onto an image starting at 
   *          (orig_x, orig_y)
   */
  void drawGraph(cv::Mat &image, const Graph& graph,
      uint32_t orig_x, uint32_t orig_y, bool put_text) {

    Graph::vertex_iterator vi, vend;
    size_t count = 0;
    for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {

      // Draw this vertex
      Point2f location = graph[*vi].location;
      size_t vertex_size = 3; // + graph[*vi].pixels / 10;
      cv::Point vertex_loc(orig_x + (uint32_t)location.x, 
          orig_y + (uint32_t)location.y);
      cv::circle(image, vertex_loc, vertex_size, cv::Scalar(0,0,255), -1);
      if (put_text) {
        cv::Point text_loc = vertex_loc + cv::Point(4,4);
        cv::putText(image, boost::lexical_cast<std::string>(count), text_loc,
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(0,0,255), 1, CV_AA);
      }

      // Draw the edges from this vertex
      Graph::adjacency_iterator ai, aend;
      for (boost::tie(ai, aend) = boost::adjacent_vertices(
            (Graph::vertex_descriptor)*vi, graph); 
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

  void writeGraphToFile(const std::string &filename, 
      const Graph& graph, const nav_msgs::MapMetaData& info) {

    std::map<Graph::vertex_descriptor, size_t> vertex_map;
    size_t count = 0;
    Graph::vertex_iterator vi, vend;
    for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
      vertex_map[*vi] = count;
      count++;
    }

    count = 0;
    YAML::Emitter out;
    out << YAML::BeginSeq;
    for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
      out << YAML::BeginMap;
      Point2f pxl_loc = graph[*vi].location;
      Point2f real_loc = toMap(pxl_loc, info);
      out << YAML::Key << "id" << YAML::Value << count;
      out << YAML::Key << "x" << YAML::Value << real_loc.x;
      out << YAML::Key << "y" << YAML::Value << real_loc.y;
      out << YAML::Key << "edges" << YAML::Value << YAML::BeginSeq;
      Graph::adjacency_iterator ai, aend;
      for (boost::tie(ai, aend) = boost::adjacent_vertices(
            (Graph::vertex_descriptor)*vi, graph); 
          ai != aend; ++ai) {
        out << vertex_map[*ai];
      }
      out << YAML::EndSeq;
      out << YAML::EndMap;
      count++;
    }
    out << YAML::EndSeq;

    std::ofstream fout(filename.c_str());
    fout << out.c_str();
    fout.close();
  }

  void readGraphFromFile(const std::string &filename, 
      const nav_msgs::MapMetaData& info, Graph& graph) {

    std::vector<std::pair<float, float> > vertices;
    std::vector<std::vector<size_t> > edges;

    std::ifstream fin(filename.c_str());
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);
    for (size_t i = 0; i < doc.size(); ++i) {
      float x, y;
      doc[i]["x"] >> x;
      doc[i]["y"] >> y;
      vertices.push_back(std::make_pair(x, y));
      std::vector<size_t> v_edges;
      const YAML::Node& edges_node = doc[i]["edges"];
      for (size_t j = 0; j < edges_node.size(); ++j) {
        size_t t;
        edges_node[j] >> t;
        if (t > i) { // Only add edge one way
          v_edges.push_back(t);
          // std::cout << "Add edge: " << i << " -> " << t << std::endl;
        }
      }
      edges.push_back(v_edges);
    }
    fin.close();

    // Construct the graph object
    for (size_t i = 0; i < vertices.size(); ++i) {
      Graph::vertex_descriptor vi = boost::add_vertex(graph);
      Point2f real_loc (vertices[i].first, vertices[i].second);
      Point2f pxl_loc (toGrid(real_loc, info));
      graph[vi].location = pxl_loc;
      graph[vi].pixels = 0; // Not saved to file as of yet
    }

    for (size_t i = 0; i < edges.size(); ++i) {
      for (size_t j = 0; j < edges[i].size(); ++j) {
        Graph::vertex_descriptor vi,vj;
        vi = boost::vertex(i, graph);
        vj = boost::vertex(edges[i][j], graph);
        Graph::edge_descriptor e; bool b;
        boost::tie(e,b) = boost::add_edge(vi, vj, graph);
        graph[e].weight = 
          topological_mapper::getMagnitude(graph[vi].location - graph[vj].location);
      }
    }

    std::cout << "Read graph with " << vertices.size() << " vertices." << std::endl;
  }

  Point2f getLocationFromGraphId(int idx, const Graph& graph) {
    Graph::vertex_descriptor i = boost::vertex(idx, graph);
    
    return Point2f(graph[i].location.x, graph[i].location.y);
  }

  size_t getClosestIdOnGraph(const Point2f &point, 
      const Graph &graph, double threshold) {
    Graph::vertex_iterator vi, vend;
    size_t count = 0, min_idx = -1;
    float min_distance = std::numeric_limits<float>::max(); 
    for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
      Point2f location = graph[*vi].location;
      if (topological_mapper::getMagnitude(point - location) <= min_distance) {
        min_distance =
          topological_mapper::getMagnitude(point - location);
        min_idx = count;
      }
      count++;
    }
    if (min_distance < threshold || threshold == 0.0) {
      return min_idx;
    } else {
      return -1;
    }
  }

  size_t getClosestIdOnGraphFromEdge(const Point2f& point, 
      const Graph &graph, size_t prev_graph_id) {

    boost::property_map<Graph, boost::vertex_index_t>::type 
        indexmap = boost::get(boost::vertex_index, graph);

    Graph::vertex_descriptor prev_vertex = boost::vertex(prev_graph_id, graph);
    Point2f location = graph[prev_vertex].location;

    size_t min_idx = -1;
    float min_distance = std::numeric_limits<float>::max();
    Point2f other_location;

    Graph::adjacency_iterator ai, aend;
    for (boost::tie(ai, aend) = boost::adjacent_vertices(prev_vertex, graph); 
        ai != aend; ++ai) {
      Point2f location2 = graph[*ai].location;

      float distance = topological_mapper::minimumDistanceToLineSegment(
           location, location2, point);
      if (distance < min_distance) {
        other_location = location2;
        min_distance = distance;
        min_idx = indexmap[*ai]; 
      }
    }

    if (getMagnitude(point - location) < getMagnitude(point - other_location)) {
      return prev_graph_id;
    } else {
      return min_idx;
    }
  }

  void getShortestPath(Graph &graph, size_t start_idx,
      size_t goal_idx, std::vector<size_t> &path_from_goal) {

    // Perform Dijakstra from start_idx
    std::vector<Graph::vertex_descriptor> 
      p(boost::num_vertices(graph));
    std::vector<double> d(boost::num_vertices(graph));
    Graph::vertex_descriptor s = 
      boost::vertex(start_idx, graph);

    boost::property_map<Graph, boost::vertex_index_t>::type 
        indexmap = boost::get(boost::vertex_index, graph);
    boost::property_map<
      Graph, 
      double Edge::*
    >::type weightmap = boost::get(&Edge::weight, graph);
    boost::dijkstra_shortest_paths(graph, s, &p[0], &d[0], weightmap, indexmap, 
                              std::less<double>(), boost::closed_plus<double>(), 
                              (std::numeric_limits<double>::max)(), 0,
                              boost::default_dijkstra_visitor());

    // Look up the parent chain from the goal vertex to the start vertex
    path_from_goal.clear();

    Graph::vertex_descriptor g = 
      boost::vertex(goal_idx, graph);
    while (indexmap[p[g]] != start_idx) {
      path_from_goal.push_back(indexmap[p[g]]);
      g = p[g];
    }
    path_from_goal.push_back(start_idx);

  }
  
} /* topological_mapper */
