#include <bwi_exp1_solver/heuristic_solver.h>
#include <topological_mapper/map_utils.h>
#include <topological_mapper/graph_utils.h>

HeuristicSolver::HeuristicSolver(const nav_msgs::OccupancyGrid& map, 
    const topological_mapper::Graph& graph, 
    bool allow_robot_current_idx) : map_(map), graph_(graph),
  goal_idx_(goal_idx), allow_robot_current_idx_(allow_robot_current_idx) {}

  HeuristicSolver::~HeuristicSolver() {}

  void HeuristicSolver::computePolicy() {}
  void loadPolicy(const std::string& file) {}
  void savePolicy(const std::string& file) {
    std::ofstream fout(file.c_str());
    fout.close();
  }

bwi_exp1::Action getBestAction(const bwi_exp1::State2& state) const {

  if (state.current_robot_status == DIR_UNASSIGNED) {
    // Find shortest path to goal. Point in direction of this path
    std::vector<size_t> path_from_goal;
    topological_mapper::getShortestPath(
        graph_, goal_idx_, state.graph_id, path_from_goal);
    std::cout << "CurId: " << state.graph_id
      << ", 1: " << path_from_goal[path_from_goal.size() - 1]
      << ", 2: " << path_from_goal[path_from_goal.size() - 2] << std::endl;
    return bwi_exp1::Action(DIRECT_PERSON, 
        path_from_goal[path_from_goal.size() - 2]);
  }

  if (state.current_robot_status != NO_ROBOT) {
    // Wait for person to transition out of this state. Do nothin
    return bwi_exp1::Action(DO_NOTHING, 0);
  }

  // Otherwise, see if we can place a robot
  if (state.num_robots_left == 0 ||
      state.graph_id == goal_idx_ || 
      state.next_robot_location != NO_ROBOT) {
    // Already placed all available robots or no need to
    return bwi_exp1::Action(DO_NOTHING, 0);
  }

  // Given the current graph id of the person and the direction the person
  // is moving in, compute the expected forward locations of the person
  std::vector<size_t> states;
  size_t current_id = state.graph_id;
  float current_direction = state.direction;

  while(true) {

    states.push_back(current_id);

    // Compute all adjacent vertices from this location
    Graph::vertex_descriptor vd = boost::vertex(current_id, graph_);
    topological_mapper::Point2f loc = graph_[vd].location;
    std::vector<size_t> adjacent_vertices;
    topological_mapper::getAdjacentVertices(
        current_id, graph_, adjacent_vertices);

    // Check vertex that has most likely transition
    size_t next_vertex = (size_t)-1;
    float next_vertex_closeness = M_PI / 4;
    float next_angle = 0;
    for (std::vector<size_t>::const_iterator av = adjacent_vertices.begin();
        av != adjacent_vertices.end(); ++av) {
      Graph::vertex_descriptor next_vd = boost::vertex(*av, graph_);
      topological_mapper::Point2f next_loc = graph_[next_vd].location;
      float angle = atan2f((next_loc-loc).y, (next_loc-loc).x);
      // Wrap angle around direction
      while (angle <= current_direction - M_PI) angle += 2 * M_PI;
      while (angle > current_direction + M_PI) angle -= 2 * M_PI;
      // Take difference
      float closeness = fabs(angle - current_direction);
      if (closeness < next_vertex_closeness) {
        next_vertex = *av;
        next_vertex_closeness = closeness;
        next_angle = angle; 
      }
    }

    if (next_vertex == (size_t)-1) {
      // No close vertices found, break
      break;
    }

    // get normalize direction and id
    current_direction = atan2f(sinf(next_angle), cosf(next_angle)); 
    current_id = next_vertex;
  }

  // Now for each state in the forward path, see which is the closest
  size_t min_graph_idx = (size_t) -1;
  float min_distance = std::numeric_limits<float>::max();
  for (std::vector<size_t>::iterator si = states.begin(); 
      si != states.end(); ++si) {
    std::vector<size_t> path_from_goal;
    topological_mapper::getShortestPath(
        graph_, goal_idx_, *si, path_from_goal);
    path_from_goal.insert(path_from_goal.begin(), goal_idx_);
    float distance = 0;
    for (size_t pp = 0; pp < path_from_goal.size() - 1; ++pp) {
      Graph::vertex_descriptor vd1 = 
        boost::vertex(path_from_goal[pp], graph_);
      Graph::vertex_descriptor vd2 = 
        boost::vertex(path_from_goal[pp + 1], graph_);
      distance += topological_mapper::getMagnitude(
          graph_[vd1].location - graph_[vd2].location);
    }
    if (distance < min_distance) {
      min_graph_idx = *si;
      min_distance = distance;
    }
  }

  if (min_graph_idx != state.graph_id || allow_robot_current_idx) {
    return bwi_exp1::Action(PLACE_ROBOT, min_graph_idx);
  }

  // This means that placing a robot on the current vertex is not allowed, and 
  // the current vertex is the closest vertex to the goal on the forward path
  return bwi_exp1::Action(DO_NOTHING, 0);

}

