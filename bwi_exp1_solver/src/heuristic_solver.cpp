#include <fstream>
#include <bwi_exp1_solver/heuristic_solver.h>
#include <topological_mapper/map_utils.h>
#include <topological_mapper/point_utils.h>

using namespace bwi_exp1;

HeuristicSolver::HeuristicSolver(const nav_msgs::OccupancyGrid& map, 
    const topological_mapper::Graph& graph, int goal_idx,
    bool allow_robot_current_idx) : map_(map), graph_(graph),
  goal_idx_(goal_idx), allow_robot_current_idx_(allow_robot_current_idx) {}

  HeuristicSolver::~HeuristicSolver() {}

  void HeuristicSolver::computePolicy() {}
  void HeuristicSolver::loadPolicy(const std::string& file) {}
  void HeuristicSolver::savePolicy(const std::string& file) {
    std::ofstream fout(file.c_str());
    fout.close();
  }

Action HeuristicSolver::getBestAction(const bwi_exp1::State2& state) const {

  if (state.current_robot_status == DIR_UNASSIGNED) {
    // Find shortest path to goal. Point in direction of this path
    std::vector<size_t> path_from_goal;
    topological_mapper::getShortestPathWithDistance(
        goal_idx_, state.graph_id, path_from_goal, graph_);
    return bwi_exp1::Action(DIRECT_PERSON, path_from_goal[0]);
  }

  if (state.current_robot_status != NO_ROBOT) {
    // Wait for person to transition out of this state. Do nothin
    return bwi_exp1::Action(DO_NOTHING, 0);
  }

  // Otherwise, see if we can place a robot. Placing robots is only allowed:
  if (state.num_robots_left == 0 || 
      state.graph_id == goal_idx_ || 
      state.visible_robot_location != NO_ROBOT) {
    // Already placed all available robots or no need to
    return bwi_exp1::Action(DO_NOTHING, 0);
  }

  // Given the current graph id of the person and the direction the person
  // is moving in, compute the expected forward locations of the person
  std::vector<size_t> states;
  size_t current_id = state.graph_id;
  float current_direction = getAngleInRadians(state.direction);
  topological_mapper::Point2f start_location = 
    topological_mapper::getLocationFromGraphId(state.graph_id, graph_); 

  /* std::cout << "Forward path: "; */
  while(true) {

    states.push_back(current_id);

    // Compute all adjacent vertices from this location
    topological_mapper::Graph::vertex_descriptor vd = 
      boost::vertex(current_id, graph_);
    topological_mapper::Point2f loc = graph_[vd].location;
    std::vector<size_t> adjacent_vertices;
    topological_mapper::getAdjacentNodes(
        current_id, graph_, adjacent_vertices);

    // Check vertex that has most likely transition
    size_t next_vertex = (size_t)-1;
    float next_vertex_closeness = M_PI / 4;
    float next_angle = 0;
    for (std::vector<size_t>::const_iterator av = adjacent_vertices.begin();
        av != adjacent_vertices.end(); ++av) {
      topological_mapper::Graph::vertex_descriptor next_vd = 
        boost::vertex(*av, graph_);
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

    topological_mapper::Point2f next_location = 
      topological_mapper::getLocationFromGraphId(next_vertex, graph_); 
    bool next_visible = 
      topological_mapper::locationsInDirectLineOfSight(                    
          start_location, next_location, map_);
    bool next_close = 
      topological_mapper::getMagnitude(next_location - start_location) <
      (25.0 / map_.info.resolution);

    if (!next_visible || !next_close) {
      // The most probable location in path is no longer visible, hence no longer
      // probable
      break;
    }

    // get normalize direction and id
    current_direction = atan2f(sinf(next_angle), cosf(next_angle)); 
    current_id = next_vertex;
  }

  // If the current vertex can't be used for robot placement, remove it from the
  // set
  if (!allow_robot_current_idx_) {
    // The first one is the current location
    states.erase(states.begin());
  }

  // Now for each state in the forward path, see which is the closest
  size_t min_graph_idx = (size_t) -1;
  float min_distance = std::numeric_limits<float>::max();
  for (std::vector<size_t>::iterator si = states.begin(); 
      si != states.end(); ++si) {
    std::vector<size_t> path_from_goal;
    float distance = topological_mapper::getShortestPathWithDistance(goal_idx_,
        *si, path_from_goal, graph_);
    if (distance < min_distance) {
      min_graph_idx = *si;
      min_distance = distance;
    }
  }

  if (min_graph_idx != goal_idx_ && min_graph_idx != (size_t)-1) {
    // The user is expected to pass through the goal
    return bwi_exp1::Action(PLACE_ROBOT, min_graph_idx);
  }

  // This means that placing a robot on the current vertex is not allowed, and 
  // the current vertex is the closest vertex to the goal on the forward path
  return bwi_exp1::Action(DO_NOTHING, 0);

}

