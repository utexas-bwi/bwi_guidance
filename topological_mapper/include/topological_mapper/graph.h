/**
 * \file  graph.h
 * \brief  Contains some simple data structures for holding the graph
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
 * $ Id: 03/04/2013 04:15:26 PM piyushk $
 *
 **/

#ifndef GRAPH_E8QGZKSM
#define GRAPH_E8QGZKSM

#include <boost/lexical_cast.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/labeled_graph.hpp>
#include <topological_mapper/structures/point.h>

#include <opencv/cv.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace topological_mapper {

  // Graph
  struct Vertex {
    Point2f location;
    double pixels;
  };

  // Edge
  struct Edge {
    double weight;
  };

  //Define the graph using those classes
  typedef boost::adjacency_list<
    boost::vecS, boost::vecS, boost::undirectedS, Vertex, Edge
  > Graph;

  /**
   * \brief   draws the given graph onto an image starting at 
   *          (orig_x, orig_y)
   */
  void drawGraph(cv::Mat &image, const Graph& graph,
      uint32_t orig_x = 0, uint32_t orig_y = 0) {

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
      Point2f real_loc;
      real_loc.x = info.origin.position.x + 
          info.resolution * pxl_loc.x;
      real_loc.y = info.origin.position.y + 
          info.resolution * pxl_loc.y;
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
        if (t > j) {
          v_edges.push_back(t);
        }
      }
      edges.push_back(v_edges);
    }
    fin.close();

    // Construct the graph object
    for (size_t i = 0; i < vertices.size(); ++i) {
      Graph::vertex_descriptor vi = boost::add_vertex(graph);
      Point2f real_loc, pxl_loc;
      real_loc.x = vertices[i].first;
      real_loc.y = vertices[i].second;
      pxl_loc.x = (real_loc.x - info.origin.position.x) / 
          info.resolution;
      pxl_loc.y = (real_loc.y - info.origin.position.y) / 
          info.resolution;
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
            sqrt(pow(graph[vi].location.x - graph[vj].location.x, 2) +
                 pow(graph[vi].location.y - graph[vj].location.y, 2));
      }
    }
  }

} /* topological_mapper */

#endif /* end of include guard: GRAPH_E8QGZKSM */
