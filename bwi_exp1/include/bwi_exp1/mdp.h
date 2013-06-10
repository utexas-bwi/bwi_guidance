#ifndef MDP_XUHAC93B
#define MDP_XUHAC93B

#include <vector>
#include <stdint.h>

#define GAMMA 0.98

namespace bwi_exp1 {

  static const double GAMMA = 0.98;
  static const size_t NUM_DIRECTIONS = 16;
  static const size_t MAX_ROBOTS = 5;

  enum ActionType {
    DO_NOTHING = 0,
    PLACE_ROBOT = 1
  };

  struct Action {
    ActionType type;
    size_t robot_id; // used in conjunction with type = PLACE_ROBOT
  };

  struct State {

    size_t graph_id; // ~100
    size_t direction; // 0 to NUM_DIRECTIONS - 1
    size_t num_robots_left; // 0 to MAX_ROBOTS

    // size_t direction_at_current; // -1 to 7, -1 for no robot
    // std::vector<size_t> frontier;
    // size_t extra_visible_robot; // sizeof(frontier) + 1 (for no extra robot)
    // size_t direction_extra_robot;

    std::vector<Action> actions;
    std::vector<size_t> next_states;

  };

  inline size_t getStateSpaceSize(size_t num_vertices) {
    return num_vertices * NUM_DIRECTIONS * (MAX_ROBOTS + 1);
  }

  inline size_t constructStateIndex(size_t graph_id, size_t direction, 
      size_t num_robots_left) {

    return graph_id * NUM_DIRECTIONS * (MAX_ROBOTS + 1) + 
      direction * (MAX_ROBOTS + 1) +
      num_robots_left;
  }

  void populateStateSpace(const topological_mapper::Graph &graph, 
      std::vector<State>& state_space, size_t goal_idx);

  void getTransitionProbabilities(const State& state, const Action& action, 
      std::vector<double>& p);

}

#endif /* end of include guard: MDP_XUHAC93B */
