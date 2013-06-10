#include 

#include <bwi_exp1/mdp.h>
#include <topological_mapper/graph.h>

namespace bwi_exp1 {

  class WeightedVertex {
   public:
    size_t vertex;
    int8_t weight;
    WeightedVertex(size_t v, int8_t w) : vertex(v), weight(w) {}
    bool operator==(const WeightedVertex& l, const WeightedVertex& r) {
      return l.vertex == r.vertex;
    }
  };

  struct weightedVertexComparator : 
      binary_function <WeightedVertex, WeightedVertex, bool> {
    inline bool operator() (const WeightedVertex& x, const WeightedVertex& y) const {
      return x.weight < y.weight;
    }
  };

  typedef WeightedVertexHeap boost::heap::fibonacci_heap<
    WeightedVertex, 
    boost::heap::compare<weightedVertexComparator>
  >;
  typedef topological_mapper::Graph Graph;

  /* not available publicly */
  void getVisibleVertices(const topological_mapper::Graph& graph, size_t src,
      size_t current, std::vector<size_t>& frontier, int8_t depth) {
    topological_mapper::Graph::vertex_descriptor vd = boost::vertex(src, graph);

    Graph::adjacency_iterator ai, aend;
    for (boost::tie(ai, aend) = boost::adjacent_vertices(
          (Graph::vertex_descriptor)*vi, graph); 
        ai != aend; ++ai) {
      
      Point2f location2 = graph[*ai].location;
    }

  }

  void populateStateSpace(const Graph &graph, 
      std::vector<State>& state_space) {

    size_t num_vertices = boost::num_vertices(graph);
    boost::property_map<Graph, boost::vertex_index_t>::type 
        indexmap = boost::get(boost::vertex_index, graph);
    for (size_t i = 0; i < num_vertices; ++i) { //graph_id

      Graph::vertex_descriptor source_v = boost::vertex(i, graph);
      topological_mapper::Point2f source_loc = graph[source_v].location;

      // Compute frontier at this vertex. 
      State state;
      WeightedVertexHeap heap;
      heap.push(WeightedVertex(i, FRONTIER_LOOKAHEAD));

      while(!heap.empty()) {
        const WeightedVertex &wv = heap.top();
        
        if (wv.weight == 0) {
          state.frontier.push_back(wv.vertex);
        } else {

          Graph::vertex_descriptor v = boost::vertex(wv.vertex, graph);
          Graph::adjacency_iterator ai, aend;
          for (boost::tie(ai, aend) = boost::adjacent_vertices(v, graph); 
              ai != aend; ++ai) {
            Point2f loc = graph[*ai].location;
          }


        }

        heap.pop();
      }
      getVisibleVertices(graph, i, i, state.frontier, heap, depth);

    }
  }
}
