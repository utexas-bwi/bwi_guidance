#ifndef MDP_XUHAC93B
#define MDP_XUHAC93B

#include <vector>
#include <stdint.h>

#define FRONTIER_LOOKAHEAD 3
#define GAMMA 0.98

namespace bwi_exp1 {

  struct Transition {
    size_t state;
    size_t reward;
    size_t value;
  };

  struct State {

    size_t graph_id;
    size_t direction; // 0 to 7
    size_t direction_at_current; // -1 to 7, -1 for no robot

    std::vector<size_t> frontier;
    size_t extra_visible_robot; // sizeof(frontier) + 1 (for no extra robot)
    size_t direction_extra_robot;

    std::vector<Transition> next;

  };

  void populateStateSpace(const topological_mapper::Graph &graph, 
      std::vector<State>& state_space);

}

#endif /* end of include guard: MDP_XUHAC93B */
