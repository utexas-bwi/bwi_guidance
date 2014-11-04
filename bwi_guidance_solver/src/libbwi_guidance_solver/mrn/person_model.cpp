#include <boost/foreach.hpp>
#include <cassert>
#include <cmath>
#include <fstream>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

#include <bwi_guidance_solver/mrn/common.h>
#include <bwi_guidance_solver/mrn/person_model.h>
#include <bwi_mapper/point_utils.h>
#include <bwi_mapper/map_utils.h>

namespace bwi_guidance_solver {

  namespace mrn {
    
    PersonModel::PersonModel(const bwi_mapper::Graph &graph, 
                             const nav_msgs::OccupancyGrid &map, 
                             int goal_idx, 
                             const MotionModel::Ptr &motion_model,
                             const HumanDecisionModel::Ptr &human_decision_model,
                             const TaskGenerationModel::Ptr &task_generation_model,
                             const Params &params) :
      graph_(graph), 
      map_(map), 
      goal_idx_(goal_idx), 
      motion_model_(motion_model), 
      human_decision_model_(human_decision_model), 
      task_generation_model_(task_generation_model),
      params_(params) {

        num_vertices_ = boost::num_vertices(graph_);
        computeAdjacentVertices(adjacent_vertices_map_, graph_);

        computeShortestPath(shortest_distances_, shortest_paths_, graph_);
      }

    void PersonModel::getActionsAtState(const State& state, 
                                        std::vector<Action>& actions) {
      int action_counter = 0;

      // Wait is allowed at all times.
      actions.push_back(Action(WAIT, 0, 0));
      ++action_counter;

      if (state.assist_type == LEAD_PERSON) {
        // Cannot execute any other action other than wait here, so return.
        return;
      }

      // Allow robots to be assigned/released at any given location. 
      for (unsigned int robot = 0; robot < state.robots.size(); ++robot) {
        if (state.robots[robot].help_destination == NONE) {
          actions.resize(action_counter + num_vertices_);
          for (unsigned int node = 0; node < num_vertices_; ++node) {
            actions[action_counter] = Action(ASSIGN_ROBOT, robot, node);
            ++action_counter;
          }
        } else {
          actions.push_back(Action(RELEASE_ROBOT, robot));
          ++action_counter;
        }
      }

      // Allow leading/guiding via all colocated robots.
      std::vector<int> robot_ids;
      getColocatedRobotIds(state, robot_ids);
      for (unsigned int robot_id_num = 0; robot_id_num < robot_ids.size(); ++robot_id_num) {
        int robot = robot_ids[robot_id_num];
        actions.resize(action_counter + 2 * adjacent_vertices_map_[state.loc_node].size());
        for (unsigned int adj = 0; adj < adjacent_vertices_map_[state.loc_node].size(); ++adj) {
          actions[action_counter] = Action(DIRECT_PERSON, robot, adjacent_vertices_map_[state.loc_node][adj]);
          ++action_counter;
          actions[action_counter] = Action(LEAD_PERSON, robot, adjacent_vertices_map_[state.loc_node][adj]);
          ++action_counter;
        }
      }
    }

    void PersonModel::getFirstAction(const State &state, Action &action) {
      // This function cannot be optimized without maintaining state. Only present for legacy
      // reasons. Do not call it! For this reason, here is a very suboptimal implementation.
      std::vector<Action> actions;
      getActionsAtState(state, actions);
      action = actions[0];
    }

    bool PersonModel::getNextAction(const State &state, Action &action) {
      // This function cannot be optimized without maintaining state. Only present for legacy
      // reasons. Do not call it!
      std::vector<Action> actions;
      getActionsAtState(state, actions);
      for (unsigned int action_id = 0; action_id < actions.size() - 1; ++action_id) {
        if (actions[action_id] == action) {
          action = actions[action_id + 1];
          return true;
        }
      }
      return false;
    }

    void PersonModel::getAllActions(const State &state, std::vector<Action>& actions) {
      getActionsAtState(state, actions);
    }

    void PersonModel::takeAction(const State &state, 
                                 const Action &action, 
                                 float &reward, 
                                 State &next_state, 
                                 bool &terminal, 
                                 int &depth_count, 
                                 boost::shared_ptr<RNG> rng) {

      float unused_utility_loss, unused_time_loss;
      std::vector<State> unused_frame_vector;
      takeAction(state, 
                 action, 
                 reward, 
                 next_state, 
                 terminal, 
                 depth_count, 
                 rng,
                 unused_time_loss, 
                 unused_utility_loss, 
                 unused_frame_vector);
    }

    void PersonModel::takeAction(const State &state, 
                                 const Action &action, 
                                 float &reward, 
                                 State &next_state, 
                                 bool &terminal, 
                                 int &depth_count,
                                 boost::shared_ptr<RNG> &rng,
                                 float &time_loss,
                                 float &utility_loss,
                                 std::vector<State> &frame_vector) {
      if (action.type != WAIT) {
        next_state = state;
        if (action.type == ASSIGN_ROBOT) {
          next_state.robots[action.robot_id].help_destination = action.node;
        } else if (action.type == RELEASE_ROBOT) {
          next_state.robots[action.robot_id].help_destination = NONE;
        } else if (action.type == DIRECT_PERSON) {
          next_state.assist_type = DIRECT_PERSON;
          next_state.assist_loc = action.node;
        } else {
          next_state.assist_type = LEAD_PERSON;
          next_state.assist_loc = action.node;
          next_state.robots[action.robot_id].help_destination = action.node;
        }
        utility_loss = 0.0f;
        time_loss = 0.0f;
        reward = 0.0f;
        frame_vector.clear();
        if (params_.frame_rate != 0.0f) {
          frame_vector.push_back(next_state);
        }
      } else {

        next_state = state;
        int next_node = human_decision_model_->GetNextNode(state, *rng);

        float total_time;
        frame_vector.clear();
        if (params_.frame_rate == 0.0f) {
          motion_model_->move(next_state, next_node, task_generation_model_, *rng, total_time); 
        } else {
          total_time = 0.0f;
          float step_time;
          while (!motion_model_->move(next_state, next_node, task_generation_model_, 
                                      *rng, step_time, 1.0f / params_.frame_rate)) {
            total_time += step_time;
            frame_vector.push_back(next_state);
          }
        }

        // After the robots have been moved, let's compute the distance to destination again to compute the utility loss
        utility_loss = 0.0f;
        // Compute the current distance to destination so that we can compute the utility loss later.
        std::vector<float> distance_to_service_destination_before_action(state.robots.size(), 0);
        for(unsigned int i = 0; i < state.robots.size(); ++i) {
          const RobotState &robot = state.robots[i];
          if (robot.help_destination != NONE) {
          }
        }

        time_loss = total_time;

        for(unsigned int i = 0; i < next_state.robots.size(); ++i) {
          const RobotState &orig_robot = state.robots[i];
          const RobotState &robot = next_state.robots[i];
          if (robot.help_destination != NONE) {
            float distance_to_service_destination_before_action = 
              getTrueDistanceTo(orig_robot.loc_u, orig_robot.loc_v, orig_robot.loc_p, orig_robot.tau_d, shortest_distances_);
            float distance_to_service_destination = 
              getTrueDistanceTo(robot.loc_u, robot.loc_v, robot.loc_p, robot.tau_d, shortest_distances_);
            float extra_distance_to_service_destination = 
              distance_to_service_destination - distance_to_service_destination_before_action;
            float extra_time_to_service_destination = 
              extra_distance_to_service_destination / motion_model_->getRobotSpeed();
            float utility_loss_per_robot = (extra_time_to_service_destination + time_loss) * robot.tau_u; 
            utility_loss += utility_loss_per_robot;
          }
        }

        // Compute reward
        reward = -(time_loss + utility_loss); 
      }
      
      depth_count = (action.type != WAIT) ? 0 : lrint(time_loss); 
      terminal = (next_state.loc_node == goal_idx_);
    }

    void PersonModel::getColocatedRobotIds(const State& state, std::vector<int> &robot_ids) {
      // Figure out if there is a robot at the current position
      for (int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {
        if ((state.robots[robot_id].help_destination == state.loc_node) &&
            isRobotExactlyAt(state.robots[robot_id], state.loc_node)) {
          robot_ids.push_back(robot_id);
        }
      }
    }

    void PersonModel::drawState(const State& state, cv::Mat& image) {

      std::vector<std::pair<cv::Point2f, cv::Scalar> > draw_robots;
      std::vector<std::vector<SquareToDraw> > draw_robot_destinations;
      std::vector<std::vector<std::vector<LineToDraw> > > draw_lines;

      draw_lines.resize(num_vertices_);
      draw_robot_destinations.resize(num_vertices_);
      for (int i = 0; i < num_vertices_; ++i) {
        draw_lines[i].resize(num_vertices_);
      }

      for (int r = 0; r < state.robots.size(); ++r) {
        RobotState robot = state.robots[r];
        cv::Scalar color(0, 64 + (r * 191) / state.robots.size(), 0);

        // There can be at most 2 destinations. The line to the service task destination needs to be dashed.
        std::vector<int> destinations;

        if (robot.help_destination != NONE) {
          // Draw the robot in blue.
          destinations.push_back(robot.help_destination);
          color = cv::Scalar(255, 0, 0);
        }

        if (robot.tau_d != NONE) {
          destinations.push_back(robot.tau_d);
          if (isRobotExactlyAt(robot, robot.tau_d)) {
            // Draw the robot in red since it is in the middle of performing the service task.
            color = cv::Scalar(0, 0, 255);
          }
        }

        cv::Point2f robot_pos = 
          (1.0f - robot.loc_p) * bwi_mapper::getLocationFromGraphId(robot.loc_u, graph_) + 
          (robot.loc_p) * bwi_mapper::getLocationFromGraphId(robot.loc_v, graph_);
        
        bool use_dashed_line = false;
        BOOST_FOREACH(int destination, destinations) {
          // Only draw the destination if we're not exactly at that destination.
          if (!isRobotExactlyAt(robot, destination)) {

            bool shortest_path_through_u = isShortestPathThroughLocU(robot.loc_u,
                                                                     robot.loc_v,
                                                                     robot.loc_p,
                                                                     destination,
                                                                     shortest_distances_);

            const std::vector<size_t> &shortest_path = (shortest_path_through_u) ?
              shortest_paths_[robot.loc_u][destination] : shortest_paths_[robot.loc_v][destination];
            int shortest_path_start_id = (shortest_path_through_u) ? robot.loc_u : robot.loc_v;

            LineToDraw l;
            l.priority = r;
            l.color = color;
            l.dashed = use_dashed_line;

            if (robot.loc_p != 0.0f && robot.loc_p != 1.0f) {
              if (shortest_path_through_u) {
                l.precision = 1.0f - robot.loc_p;
                draw_lines[robot.loc_v][robot.loc_u].push_back(l); 
              } else {
                l.precision = robot.loc_p;
                draw_lines[robot.loc_u][robot.loc_v].push_back(l); 
              }
            }

            if (shortest_path.size() != 0) {
              int current_node = shortest_path_start_id;
              for (int s = 0; s < shortest_path.size(); ++s) {
                int next_node = shortest_path[s];
                l.precision = 0.0f;
                draw_lines[current_node][next_node].push_back(l); 
                current_node = next_node;
              }
            }

            // Draw the destination as well.
            SquareToDraw s;
            s.color = color;
            draw_robot_destinations[destination].push_back(s);
          }
          use_dashed_line = true;
        }
        draw_robots.push_back(std::make_pair(robot_pos, color));
      }

      for (int i = 0; i < num_vertices_; ++i) {
        for (int j = i+1; j < num_vertices_; ++j) {
          std::vector<LineToDraw>& ijlines = draw_lines[i][j];  
          std::vector<LineToDraw>& jilines = draw_lines[j][i];  

          // For each line, store a standard vector of thickness based on priority
          std::set<int> priorities;
          BOOST_FOREACH(const LineToDraw& l, ijlines) {
            priorities.insert(l.priority);
          }
          BOOST_FOREACH(const LineToDraw& l, jilines) {
            priorities.insert(l.priority);
          }

          std::map<int, int> thickness_map;
          int p_count = priorities.size() - 1;
          BOOST_FOREACH(int p, priorities) {
            thickness_map[p] = 2 + 4 * p_count;
            --p_count;
          }

          int ijcounter = 0;
          int jicounter = 0;
          while (ijcounter < ijlines.size() || jicounter < jilines.size()) {
            LineToDraw* line = NULL;
            int s, e;
            if (ijcounter == ijlines.size()) {
              s = j;
              e = i;
              line = &(jilines[jicounter++]);
            } else if (jicounter == jilines.size()) {
              s = i;
              e = j;
              line = &(ijlines[ijcounter++]);
            } else if (ijlines[ijcounter].priority <= 
                       jilines[jicounter].priority) {
              s = i;
              e = j;
              line = &(ijlines[ijcounter++]);
            } else {
              s = j;
              e = i;
              line = &(jilines[jicounter++]);
            }
            // Draw this line
            cv::Point2f start_pos = 
              (1.0f - line->precision) * bwi_mapper::getLocationFromGraphId(s, graph_) + 
              line->precision * bwi_mapper::getLocationFromGraphId(e, graph_);
            if (line->dashed) {
              dashedLine(image, bwi_mapper::getLocationFromGraphId(e, graph_), start_pos, line->color, 10,
                         thickness_map[line->priority], CV_AA);
            } else {
              cv::line(image,
                       bwi_mapper::getLocationFromGraphId(e, graph_),
                       start_pos,
                       line->color, thickness_map[line->priority], CV_AA);
            }
          }
        }
      }

      for (int i = 0; i < num_vertices_; ++i) {
        int size = 18 + 8 * draw_robot_destinations[i].size();
        BOOST_FOREACH(const SquareToDraw& square, draw_robot_destinations[i]) {
          bwi_mapper::drawSquareOnGraph(image, graph_, i, square.color, 0, 0, size);
          size -= 8;
        }
      }

      cv::Point2f goal_loc = bwi_mapper::getLocationFromGraphId(goal_idx_, graph_);
      drawCheckeredFlagOnImage(image, goal_loc);

      for (int i = 0; i < draw_robots.size(); ++i) {
        drawRobotOnImage(image, draw_robots[i].first + cv::Point2f(6, 6), draw_robots[i].second); 
      }

      cv::Point2f human_pos = (1 - state.loc_p) * bwi_mapper::getLocationFromGraphId(state.loc_node, graph_);
      if (state.loc_p > 0.0f) {
        // Note that loc_prev is a bit abused here. When loc_p != 0, loc_prev actually points to the node the person
        // is transitioning to.
        human_pos += state.loc_p * bwi_mapper::getLocationFromGraphId(state.loc_prev, graph_);
      }

      // Offset for person
      human_pos.x -= 6;
      human_pos.y -= 0;
      drawPersonOnImage(image, human_pos);

      // If human is being assisted, then display the assistance type.
      if (state.assist_type != NONE) {
        // Add the offset for the robot's location.
        cv::Point2f colocated_robot_pos = human_pos + cv::Point2f(12, 6); 

        if ((state.assist_type == DIRECT_PERSON) && state.loc_p == 0.0f) {
          float orientation = bwi_mapper::getNodeAngle(state.loc_node, state.assist_loc, graph_);
          drawScreenWithDirectedArrowOnImage(image, colocated_robot_pos, orientation);
        } else if (state.assist_type == LEAD_PERSON) {
          drawScreenWithFollowMeText(image, colocated_robot_pos);
        }
      }

    }

  } /* mrn */

} /* bwi_guidance */
