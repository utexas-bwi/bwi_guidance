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
      size_t graph_id; // with PLACE ROBOT, identifies the direction pointed to
  };

  struct State {
    size_t graph_id; // ~100
    size_t direction; // 0 to NUM_DIRECTIONS - 1
    size_t num_robots_left; // 0 to MAX_ROBOTS
  };

  inline float roundWithOffset(float value, float offset) {
    return round(value + offset) - offset;
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

  inline bool isTerminalState(const State& state, size_t goal_idx) {
    return state.graph_id == goal_idx;
  }
  
  inline float getReward(const std::vector<State>& state_space, size_t from_idx,
      size_t to_idx, const Graph& graph) {

    Graph::vertex_descriptor from_v = 
      boost::vertex(state_space[from_idx].graph_id, graph);
    Graph::vertex_descriptor to_v = 
      boost::vertex(state_space[to_idx].graph_id, graph);

    // Return the cost (i.e negative) for the distance between the 2 state_ids
    return -cv::norm(graph[from_v].location - graph[to_v].location);
  }

  inline float getAngleFromStates(
      const Graph& graph, size_t graph_id, size_t next_graph_id) {

    Graph::vertex_descriptor v = boost::vertex(graph_id, graph);
    Graph::vertex_descriptor next_v = boost::vertex(next_graph_id, graph);

    return atan2f(graph[next_v].location.y - graph[v].location.y,
        graph[next_v].location.x - graph[v].location.x);
  }

  inline float getAngleFromDirection(size_t dir) {

    float max_value = ((float)GRID_SIZE - 1.0) / 2.0;

    float x = (dir % GRID_SIZE) - max_value;
    float y = (dir / GRID_SIZE) - max_value;

    return atan2f(y, x);
  }

  inline void getXYDirectionFromStates(
      const Graph& graph, size_t graph_id, size_t next_graph_id, 
      float& x, float& y) {

    float angle = getAngleFromStates(graph, graph_id, next_graph_id);
    float slope = tanf(angle);
    
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

  }
 
  size_t computeNextDirection(size_t dir, size_t graph_id, 
      size_t next_graph_id, const Graph &graph);

  void getActionsAtState(const State& state, const Graph& graph, 
      std::vector<Action>& actions);
  
  void getNextStatesAtState(const State& state, const Graph& graph, 
      std::vector<size_t>& next_states);

  void populateStateSpace(const topological_mapper::Graph &graph, 
      std::vector<State>& state_space);

  void getTransitionProbabilities(const State& state, 
      const std::vector<State>& state_space, 
      const Graph& graph, const Action& action, 
      std::vector<size_t>& next_states, std::vector<float>& probabilities);

}

#endif /* end of include guard: MDP_XUHAC93B */
