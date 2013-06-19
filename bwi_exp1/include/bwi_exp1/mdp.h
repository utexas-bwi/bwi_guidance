#ifndef MDP_XUHAC93B
#define MDP_XUHAC93B

#include <cmath>
#include <vector>
#include <stdint.h>
#include <topological_mapper/graph.h>

namespace bwi_exp1 {

  typedef topological_mapper::Graph Graph;

  const double GAMMA = 0.98;

  // Even values here will have weird random discretization errors due to an
  // inability of representing cardinal directions
  const size_t GRID_SIZE = 5;
  const size_t NUM_DIRECTIONS = GRID_SIZE * GRID_SIZE;
  const size_t MAX_ROBOTS = 5;

  enum ActionType {
    DO_NOTHING = 0,
    PLACE_ROBOT = 1
  };

  class Action {
    public:
      Action(ActionType a, size_t g) : type(a), graph_id(g) {}
      ActionType type;
      size_t graph_id; // used in conjunction with type = PLACE_ROBOT
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

  inline getActionSpaceAtSet(const State& state, const Graph& graph, 
      std::vector<Action>& actions, std::vector<size_t>& next_states) {

    // Compute the set of possible actions 
    std::vector<size_t> adjacent_idxs;
    Graph::adjacency_iterator ai, aend;
    for (boost::tie(ai, aend) = boost::adjacent_vertices(v, graph); 
        ai != aend; ++ai) {
      adjacent_idxs.push_back(indexmap[*ai]);
    }

    std::vector<Action> all_actions, no_robot_actions;
    all_actions.push_back(Action(DO_NOTHING,0));
    no_robot_actions.push_back(Action(DO_NOTHING,0));


  }

  inline float roundWithOffset(float value, float offset) {
    return round(value + offset) - offset;
  }

  inline size_t computeNextDirection(size_t dir, size_t graph_id, 
      size_t next_graph_id, const Graph &graph) {

    // Compute 1 of 16 directions from current graph_id to next_graph_id
    Graph::vertex_descriptor v = boost::vertex(graph_id, graph);
    Graph::vertex_descriptor next_v = boost::vertex(next_graph_id, graph);

    float slope = (graph[next_v].location.y - graph[v].location.y) /
        (graph[next_v].location.x - graph[v].location.x);
    float angle = atan2f(graph[next_v].location.y - graph[v].location.y,
        graph[next_v].location.x - graph[v].location.x);
    
    float x = 0, y = 0;
    float max_value = ((float)GRID_SIZE - 1.0) / 2.0;
    float offset = ((GRID_SIZE + 1) % 2) * 0.5;

    if (angle >= M_PI/4.0 && angle < 3.0*M_PI/4.0) {
      y = max_value;
      x = roundWithOffset(y/slope, offset);
    } else if (angle >= 3.0*M_PI/4.0 || angle < -3.0*M_PI/4.0) {
      x = -max_value;
      y = roundWithOffset(x*slope, offset);
    } else if (angle >= -3.0*M_PI/4.0 && angle < -M_PI/4.0) {
      y = -max_value;
      x = roundWithOffset(y/slope, offset);
    } else {
      x = max_value;
      y = roundWithOffset(x*slope, offset);
    }

    //std::cout << x << " " << y << std::endl;

    float x_curr = (dir % GRID_SIZE) - max_value;
    float y_curr = (dir / GRID_SIZE) - max_value;

    //std::cout << dir << " -> " << x_curr << " " << y_curr << std::endl;

    float x_net = x_curr / 2.0 + x;
    x_net = std::min(max_value, x_net);
    x_net = std::max(-max_value, x_net);
    float y_net = y_curr / 2.0 + y;
    y_net = std::min(max_value, y_net);
    y_net = std::max(-max_value, y_net);

    int x_idx = round(x_net + offset) - round (-max_value + offset);
    int y_idx = round(y_net + offset) - round (-max_value + offset);

    return y_idx * GRID_SIZE + x_idx;
  }

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
