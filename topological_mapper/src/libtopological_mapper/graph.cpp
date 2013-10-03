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
#include <boost/foreach.hpp>

#include <topological_mapper/graph.h>
#include <topological_mapper/map_utils.h>
#include <topological_mapper/point_utils.h>

namespace topological_mapper {

  /**
   * \brief   draws the given graph onto an image starting at 
   *          (orig_x, orig_y)
   */
  void drawGraph(cv::Mat &image, const Graph& graph,
      uint32_t orig_x, uint32_t orig_y, bool put_text, bool put_all_edges,
      std::vector<std::pair<size_t, size_t> > specific_edges) {

    Graph::vertex_iterator vi, vend;
    size_t count = 0;
    for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; 
        ++vi, ++count) {
      Point2f location = graph[*vi].location;
      // Draw the edges from this vertex
      std::vector<size_t> adj_vertices;
      getAdjacentVertices(count, graph, adj_vertices);
      BOOST_FOREACH(size_t adj_vtx, adj_vertices) {
        if (adj_vtx > count) {
          bool allow_edge = put_all_edges;
          allow_edge = allow_edge || 
            std::find(specific_edges.begin(), specific_edges.end(), 
                std::make_pair(count, adj_vtx)) != specific_edges.end();
          allow_edge = allow_edge || 
            std::find(specific_edges.begin(), specific_edges.end(), 
                std::make_pair(adj_vtx, count)) != specific_edges.end();
          if (allow_edge) {
            Point2f location2 = getLocationFromGraphId(adj_vtx, graph);
            cv::Point start_pt = 
              cv::Point(orig_x + location.x, orig_y + location.y);
            cv::Point end_pt = 
              cv::Point(orig_x + location2.x, orig_y + location2.y);
            float shift_ratio = 15.0f / cv::norm(start_pt - end_pt);
            cv::Point start_shift = start_pt + shift_ratio * (end_pt - start_pt); 
            cv::Point end_shift = end_pt + shift_ratio * (start_pt - end_pt); 
            cv::line(image, start_shift, end_shift, 
                cv::Scalar(160, 160, 255),
                2, CV_AA); // draw an anti aliased line
          }
        }
      }
    }

    count = 0;
    for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
      Point2f location = graph[*vi].location;
      // Draw this vertex
      size_t vertex_size = 3; // + graph[*vi].pixels / 10;
      cv::Point vertex_loc(orig_x + (uint32_t)location.x, 
          orig_y + (uint32_t)location.y);
      /* cv::circle(image, vertex_loc, vertex_size, cv::Scalar(0,0,255), -1); */
      if (put_text) {
        cv::Point text_loc = vertex_loc + cv::Point(-7,7);
        if (count >= 10) {
          text_loc = text_loc + cv::Point(-7,0);
        }
        cv::putText(image, boost::lexical_cast<std::string>(count), text_loc,
            cv::FONT_HERSHEY_SIMPLEX, 0.7, cvScalar(0,0,255), 2, CV_AA);
      }

      count++;
    }
  }

  void drawArrowOnGraph(cv::Mat &image, const Graph& graph, 
      std::pair<size_t, float> arrow, cv::Scalar color,
      uint32_t orig_x, uint32_t orig_y) {

    float orientation = arrow.second;
    Point2f loc = getLocationFromGraphId(arrow.first, graph);
    cv::Point node_loc(loc.x + orig_x, loc.y + orig_y); 
    cv::Point arrow_center = node_loc + 
      cv::Point(20 * cosf(orientation), 20 * sinf(orientation));

    cv::Point arrow_start = arrow_center +
      cv::Point(15 * cosf(orientation + M_PI/2), 
                15 * sinf(orientation + M_PI/2));
    cv::Point arrow_end = arrow_center -
      cv::Point(15 * cosf(orientation + M_PI/2), 
                15 * sinf(orientation + M_PI/2));

    cv::line(image, arrow_start, arrow_end, color, 2, CV_AA);

    // http://mlikihazar.blogspot.com/2013/02/draw-arrow-opencv.html
    cv::Point p(arrow_start), q(arrow_end);

    //Draw the first segment
    float angle = atan2f(p.y - q.y, p.x - q.x);
    p.x = (int) (q.x + 6 * cos(angle + M_PI/4));
    p.y = (int) (q.y + 6 * sin(angle + M_PI/4));
    cv::line(image, p, q, color, 2, CV_AA);

    //Draw the second segment
    p.x = (int) (q.x + 8 * cos(angle - M_PI/4));
    p.y = (int) (q.y + 8 * sin(angle - M_PI/4));
    cv::line(image, p, q, color, 2, CV_AA);
  }

  void drawCircleOnGraph(cv::Mat &image, const Graph& graph, 
      size_t node, cv::Scalar color,
      uint32_t orig_x, uint32_t orig_y) {
    Point2f loc = getLocationFromGraphId(node, graph);
    cv::Point circle_loc(loc.x + orig_x, loc.y + orig_y); 
    cv::circle(image, circle_loc, 15, color, 2, CV_AA); 
  }

  void drawSquareOnGraph(cv::Mat &image, const Graph& graph, 
      size_t node, cv::Scalar color,
      uint32_t orig_x, uint32_t orig_y) {
    Point2f loc = getLocationFromGraphId(node, graph);
    cv::Point square_loc(loc.x + orig_x, loc.y + orig_y); 
    cv::Rect rect(square_loc.x - 15, square_loc.y - 15, 30, 30);
    cv::rectangle(image, rect, color, 2, CV_AA); 
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

  void getAdjacentVertices(size_t graph_id, const Graph& graph,
      std::vector<size_t>& adjacent_vertices) {

    adjacent_vertices.clear();
    boost::property_map<Graph, boost::vertex_index_t>::type 
        indexmap = boost::get(boost::vertex_index, graph);
    Graph::vertex_descriptor vertex = boost::vertex(graph_id, graph);

    Graph::adjacency_iterator ai, aend;
    for (boost::tie(ai, aend) = boost::adjacent_vertices(vertex, graph); 
        ai != aend; ++ai) {
      adjacent_vertices.push_back(indexmap[*ai]);
    }

  }

  void getShortestPath(const Graph &graph, size_t start_idx,
      size_t goal_idx, std::vector<size_t> &path_from_goal) {

    topological_mapper::Graph graph_copy(graph);
    // Perform Dijakstra from start_idx
    std::vector<Graph::vertex_descriptor> 
      p(boost::num_vertices(graph_copy));
    std::vector<double> d(boost::num_vertices(graph_copy));
    Graph::vertex_descriptor s = 
      boost::vertex(start_idx, graph_copy);

    boost::property_map<Graph, boost::vertex_index_t>::type 
        indexmap = boost::get(boost::vertex_index, graph_copy);
    boost::property_map<
      Graph, 
      double Edge::*
    >::type weightmap = boost::get(&Edge::weight, graph_copy);
    boost::dijkstra_shortest_paths(graph_copy, 
        s, &p[0], &d[0], weightmap, indexmap, 
                              std::less<double>(), boost::closed_plus<double>(), 
                              (std::numeric_limits<double>::max)(), 0,
                              boost::default_dijkstra_visitor());

    // Look up the parent chain from the goal vertex to the start vertex
    path_from_goal.clear();

    Graph::vertex_descriptor g = 
      boost::vertex(goal_idx, graph_copy);
    while (indexmap[p[g]] != start_idx) {
      path_from_goal.push_back(indexmap[p[g]]);
      g = p[g];
    }
    path_from_goal.push_back(start_idx);

  }
  
} /* topological_mapper */
