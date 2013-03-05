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

#include <boost/graph/adjacency_list.hpp>
#include <topological_mapper/structures/point.h>

namespace topological_mapper {

  // Graph
  struct Vertex {
    uint32_t index;
    Point2f location;
    double pixels; // Stores area for regions and clearance for vertices
    double resolution;
  };

  struct Edge {
    uint32_t index;
  };

  //Define the graph using those classes
  typedef boost::adjacency_list<
    boost::listS, 
    boost::listS, 
    boost::undirectedS, 
    Vertex, 
    Edge
  > Graph;

  const Vertex* getVertex(const Graph &g, uint32_t index) {
    boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
    boost::tie(vi, vi_end) = boost::vertices(g);
    for (; vi != vi_end; ++vi) {
      if (g[*vi].index == index) {
        return &g[*vi];
      }
    }
    return NULL;
  }

  const Edge* getEdge(const Graph &g, uint32_t index) {
    boost::graph_traits<Graph>::edge_iterator ei, ei_end;
    boost::tie(ei, ei_end) = edges(g);
    for (; ei != ei_end; ++ei) {
      if (g[*ei].index == index) {
        return &g[*ei];
      }
    }
    return NULL;
  }

} /* topological_mapper */

