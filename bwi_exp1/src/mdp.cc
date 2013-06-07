#include <bwi_exp1/mdp.h>
#include <t

namespace bwi_exp1 {

  void getVisibleVertices(graph, i, state.frontier, depth) {
    topological_mapper::Graph::vertex_descriptor vd = 

  }

  void populateStateSpace(const topological_mapper::Graph &graph, 
      std::vector<State>& state_space) {

    size_t num_vertices = boost::num_vertices(graph);
    for (size_t i = 0; i < num_vertices; ++i) { //graph_id

      // Compute frontier at this vertex. 
      // Don't worry about directionality right now
      State state;
      getVisibleVertices(graph, i, i, state.frontier, depth);

    }
  }
}
