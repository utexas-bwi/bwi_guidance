#include <fstream>

#include <boost/foreach.hpp>

#include <bwi_guidance_solver/mrn/heuristic_solver.h>

namespace bwi_guidance_solver {

  namespace mrn {

    HeuristicSolver::HeuristicSolver(const nav_msgs::OccupancyGrid& map, const bwi_mapper::Graph& graph, int goal_idx,
                                     bool improved, float human_speed) : HeuristicSolver(map, graph, goal_idx, true,
                                                                                         0.0f, true),
                                     improved_(improved), human_speed_(human_speed) {
      human_speed_ /= map.info.resolution;
    }

    HeuristicSolver::~HeuristicSolver() {}

    void HeuristicSolver::computePolicy() {}
    void HeuristicSolver::loadPolicy(const std::string& file) {}
    void HeuristicSolver::savePolicy(const std::string& file) {
      std::ofstream fout(file.c_str());
      fout.close();
    }

    Action HeuristicSolver::getBestAction(const bwi_guidance::State& state, 
                                          const boost::shared_ptr<PersonModel>& evaluation_model) {

      // Map the current state into that of the QRR Heuristic Solver, and 
      // use that solver out of the box. Then map the action to the new domain
      // and return the mapped action

      StateQRR14 mapped_state;
      mapped_state.graph_id = state.graph_id;
      mapped_state.direction = state.direction;
      bool cur_robot_available = 
        evaluation_model->isRobotDirectionAvailable(state,
                                                    mapped_state.robot_direction);
      mapped_state.robot_direction = (cur_robot_available) ?
        mapped_state.robot_direction : NONE;
      mapped_state.num_robots_left = 5;
      mapped_state.visible_robot = NONE;

      // Figure out the vertices that cannot be reached in time
      boost::shared_ptr<std::vector<int> > blacklisted_vertices;
      blacklisted_vertices.reset(new std::vector<int>);
      blacklisted_vertices->insert(blacklisted_vertices->end(), 
                                   state.relieved_locations.begin(), state.relieved_locations.end());
      if (improved_) {
        BOOST_FOREACH(const int& vtx, visible_vertices_map_[mapped_state.graph_id]) {
          if (state.in_use_robots.size() != 0) {
            if (vtx == state.in_use_robots[0].destination) {
              continue;
            }
          }
          bool reach_in_time;
          float time_to_destination = 
            bwi_mapper::getShortestPathDistance(state.graph_id, vtx, graph_) /
            human_speed_;
          evaluation_model->selectBestRobotForTask(vtx, time_to_destination,
                                                   reach_in_time);
          if (!reach_in_time) {
            std::cout << vtx << " ";
            blacklisted_vertices->push_back(vtx);
          }
        }
        std::cout << std::endl;
      }

      // Get the best action 
      irm::Action action = HeuristicSolver::getBestAction(mapped_state,
                                                          blacklisted_vertices);
      Action mapped_action;
      switch(action.type) {
        case irm::DO_NOTHING:
          if (state.in_use_robots.size() != 0) {
            // The approach does not require a robot. Release acquired robot.
            mapped_action = Action(RELEASE_ROBOT, state.in_use_robots[0].destination, NONE);
          }
          return mapped_action;
        case irm::DIRECT_PERSON:
          mapped_action = Action(GUIDE_PERSON, mapped_state.graph_id, action.graph_id);
          return mapped_action;
        case irm::PLACE_ROBOT:
          if (state.in_use_robots.size() != 0) {
            if (state.in_use_robots[0].destination != action.graph_id) {
              // The approach wants to place a robot at a diff location, release the current one
              mapped_action = Action(RELEASE_ROBOT, state.in_use_robots[0].destination, NONE);
            } else {
              // A robot is already heading there. Carry on, nothing to do here.
            }
          } else {
            mapped_action = Action(ASSIGN_ROBOT, action.graph_id, DIR_UNASSIGNED);
          }
          return mapped_action;
      }
    }

  } /* mrn */

} /* bwi_guidance_solver */


