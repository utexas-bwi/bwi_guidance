#include <boost/foreach.hpp>
#include <cassert>
#include <cmath>
#include <fstream>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

#include <bwi_guidance_solver/mrn/person_model.h>
#include <bwi_mapper/point_utils.h>
#include <bwi_mapper/map_utils.h>

namespace bwi_guidance_solver {

  namespace mrn {
    
    PersonModel::PersonModel(const bwi_mapper::Graph &graph, 
                             const nav_msgs::OccupancyGrid &map, 
                             int goal_idx, 
                             const Params &params) :
      graph_(graph), map_(map), goal_idx_(goal_idx), params_(params) {

        params_.avg_robot_speed /= map_.info.resolution;
        params_.avg_human_speed /= map_.info.resolution;

        num_vertices_ = boost::num_vertices(graph_);
        computeAdjacentVertices(adjacent_vertices_map_, graph_);

        cacheNewGoalsByDistance();
        cacheShortestPaths();
      }

    void PersonModel::getActionsAtState(const State& state, 
                                        std::vector<Action>& actions) {
      int action_counter = 0;

      // Wait is allowed at all times.
      actions.resize(1);
      actions[action_counter] = Action(WAIT, 0, 0);
      ++action_counter;

      if (state.assist_type == LEAD_PERSON) {
        // Cannot execute any other action other than wait here, so return.
        return;
      }

      // Allow robots to be assigned/released at any given location. 
      for (unsigned int robot = 0; robot < params_.num_robots; ++robot) {
        if (state.robots[robot].help_destination == NONE) {
          actions.resize(action_counter + num_vertices_);
          for (unsigned int node = 0; node < num_vertices_; ++node) {
            actions[action_counter] = Action(ASSIGN_ROBOT, robot, node);
            ++action_counter;
          }
        } else {
          actions.push_back(Action(RELEASE_ROBOT, robot));
        }
      }

      // Allow leading/guiding via all colocated robots.
      std::vector<int> robot_ids;
      getColocatedRobotIds(state, robot_ids);
      for (unsigned int robot_id_num = 0; robot_id_num < robot_ids.size(); ++robot_id_num) {
        int robot = robot_ids[robot_id_num];
        actions.resize(action_counter + 2 * adjacent_vertices_map_[state.loc_node].size());
        for (unsigned int adj = 0; adj < 2 * adjacent_vertices_map_[state.loc_node].size(); ++adj) {
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
      return actions[0];
    }

    virtual bool getNextAction(const State &state, Action &action) {
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
          next_state.robots[action.robot].help_destination = action.node;
        } else if (action.type == RELEASE_ROBOT) {
          next_state.robots[action.robot].help_destination = NONE;
        } else if (action.type == DIRECT_PERSON) {
          next_state.assist_type = DIRECT_PERSON;
          next_state.assist_loc = action.node;
        } else {
          next_state.assist_type = LEAD_PERSON;
          next_state.assist_loc = action.node;
        }
        utility_loss = 0.0f;
        time_loss = 0.0f;
        reward = 0.0f;
      } else {

        // Compute the current distance to destination so that we can compute the utility loss later.
        std::vector<float> distance_to_service_destination_before_action(state.robots.size(), 0);
        for(unsigned int i = 0; i < state.robots.size(); ++i) {
          const RobotState &robot = state.robot[i];
          if (robot.help_destination != NONE) {
            distance_to_service_destination_before_action[i] = 
              getTrueDistanceTo(robot.loc_u, robot.loc_v, robot.loc_p, robot.tau_d);
          }
        }

        next_state = state;
        int next_node = human_model_->GetNextNode(state);

        float total_time;
        if (frame_rate_ == 0.0f) {
          transition_model_->move(next_state, next_node, task_generation_model_, rng, total_time); 
        } else {
          frame_vector.clear();
          while (!transition_model_->move(next_state, next_node, task_generation_model_, rng, total_time, 1.0f / frame_rate_)) {
            frame_vector.push_back(next_state);
          }
        }

        // After the robots have been moved, let's compute the distance to destination again to compute the utility loss
        utility_loss = 0.0f;
        for(unsigned int i = 0; i < state.robots.size(); ++i) {
          const RobotState &robot = state.robot[i];
          if (robot.help_destination != NONE) {
            float distance_to_service_destination = 
              getTrueDistanceTo(robot.loc_u, robot.loc_v, robot.loc_p, robot.tau_d);
            float distance_difference = 
              std::max(0.0f, distance_to_service_destination_before_action[i] - distance_to_service_destination);
            float time_difference = distance_difference / robot_speed_;
            float utility_loss_per_robot = time_difference * robot.tau_u; 
            utility_loss += utility_loss_per_robot;
          }
        }

        time_loss = total_time;

        // Compute reward
        reward = -(time_loss + utility_loss); 
      }
      
      depth_count = (action.type != WAIT) ? 0 : lrint(time_loss); 
      terminal = (next_state.graph_id == goal_idx_);
    }

    bool PersonModel::isAssignedRobotColocated(const State& state) {
      // Figure out if there is a robot at the current position
      for (int i = 0; i < state.in_use_robots.size(); ++i) {
        if ((state.in_use_robots[i].destination == state.graph_id) && 
            (state.in_use_robots[i].reached_destination)) {
          return true;
        }
      }
      return false;
    }

    void PersonModel::drawState(const State& state, cv::Mat& image) {

      std::vector<std::pair<cv::Point2f, cv::Scalar> > draw_circles;
      std::vector<std::vector<SquareToDraw> > draw_squares;
      std::vector<std::vector<std::vector<LineToDraw> > > draw_lines;
      draw_lines.resize(num_vertices_);
      draw_squares.resize(num_vertices_);
      for (int i = 0; i < num_vertices_; ++i) {
        draw_lines[i].resize(num_vertices_);
      }

      for (int r = 0; r < state.robots.size(); ++r) {
        RobotState robot = state.robots[r];
        cv::Scalar color(0, 128 + (r * 12345) % 128, 128 + (r * 23457) % 128);
        bool robot_in_use = false;
        std::vector<int> destinations;
        // The destination can be -1 for visualization purposes only.
        if (robot.destination != -1)
        {
          destinations.push_back(robot.destination);
        }
        bool use_dashed_line = false;
        for (int j = 0; j < state.in_use_robots.size(); ++j) {
          if (state.in_use_robots[j].robot_id == r) {
            robot_in_use = true;
            if (state.in_use_robots[j].destination != -1 && !(state.in_use_robots[j].reached_destination)) {
              destinations.insert(destinations.begin(), state.in_use_robots[j].destination);
            } else {
              use_dashed_line = true;
            }
            break;
          }
        }
        if (robot_in_use) {
          color = cv::Scalar(255, 0, 0);
        }
        cv::Point2f robot_pos; 
        if (robot.precision < 0.5f) {
          robot_pos = 
            (1.0f - robot.precision) * bwi_mapper::getLocationFromGraphId(robot.graph_id, graph_) + 
            (robot.precision) * bwi_mapper::getLocationFromGraphId(robot.other_graph_node, graph_);
        } else {
          robot_pos = 
            robot.precision * bwi_mapper::getLocationFromGraphId(robot.graph_id, graph_) + 
            (1.0f - robot.precision) * bwi_mapper::getLocationFromGraphId(robot.other_graph_node, graph_);
        }
        BOOST_FOREACH(int destination, destinations) {
          changeRobotDirectionIfNeeded(robot, 0, destination);
          std::vector<size_t>* shortest_path = NULL;
          int shortest_path_start_id;
          if (robot.precision < 0.5f) {
            shortest_path = &(shortest_paths_[robot.other_graph_node][destination]);
            shortest_path_start_id = robot.other_graph_node;
            LineToDraw l;
            l.priority = r;
            l.precision = robot.precision;
            l.color = color;
            l.dashed = use_dashed_line;
            draw_lines[robot.graph_id][robot.other_graph_node].push_back(l); 
          } else {
            shortest_path = &(shortest_paths_[robot.graph_id][destination]);
            shortest_path_start_id = robot.graph_id;
            LineToDraw l;
            l.priority = r;
            l.precision = robot.precision;
            l.color = color;
            l.dashed = use_dashed_line;
            draw_lines[robot.other_graph_node][robot.graph_id].push_back(l); 
          }
          if (shortest_path->size() != 0) {
            int current_node = shortest_path_start_id;
            for (int s = 0; s < shortest_path->size(); ++s) {
              int next_node = (*shortest_path)[s];
              LineToDraw l;
              l.priority = r;
              l.precision = 0.0f;
              l.color = color;
              l.dashed = use_dashed_line;
              draw_lines[current_node][next_node].push_back(l); 
              current_node = next_node;
            }
          }
          SquareToDraw s;
          s.color = color;
          draw_squares[destination].push_back(s);
          use_dashed_line = true;
        }
        draw_circles.push_back(std::make_pair(robot_pos, color));
      }

      // Draw the person's destination
      // SquareToDraw s;
      // s.color = cv::Scalar(0,0,255);
      // draw_squares[goal_idx_].push_back(s);

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
          // if (thickness != 2) {
          //   std::cout << "ST: " << thickness << std::endl;
          // }
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
        int size = 18 + 8 * draw_squares[i].size();
        BOOST_FOREACH(const SquareToDraw& square, draw_squares[i]) {
          bwi_mapper::drawSquareOnGraph(image, graph_, i, square.color, 0, 0,
                                        size);
          size -= 8;
        }
      }

      cv::Point2f goal_loc = bwi_mapper::getLocationFromGraphId(goal_idx_, graph_);
      drawCheckeredFlagOnImage(image, goal_loc);

      for (int i = 0; i < draw_circles.size(); ++i) {
        drawRobotOnImage(image, draw_circles[i].first + cv::Point2f(6, 6), draw_circles[i].second); 
      }

      cv::Point2f human_pos = 
        (1 - state.precision) * bwi_mapper::getLocationFromGraphId(state.from_graph_node, graph_) + 
        state.precision * bwi_mapper::getLocationFromGraphId(state.graph_id, graph_);

      // Offset for person
      human_pos.x -= 6;
      human_pos.y -= 0;
      drawPersonOnImage(image, human_pos);

    }

    void PersonModel::drawAction(const State& state, const Action& action, cv::Mat& image) {

      cv::Point2f colocated_robot_pos = 
        (1 - state.precision) * bwi_mapper::getLocationFromGraphId(state.from_graph_node, graph_) + 
        state.precision * bwi_mapper::getLocationFromGraphId(state.graph_id, graph_);
      colocated_robot_pos += cv::Point2f(6, 6); // Add the offset for the robot's location.

      if (action.type == DIRECT_PERSON) {
        int from_graph_id = state.from_graph_node;
        if (state.precision == 1.0f) {
          from_graph_id = state.graph_id;
        }
        float orientation = bwi_mapper::getNodeAngle(from_graph_id, action.guide_graph_id, graph_);
        drawScreenWithDirectedArrowOnImage(image, colocated_robot_pos, orientation);
      } else if (action.type == LEAD_PERSON) {
        drawScreenWithFollowMeText(image, colocated_robot_pos);
      }

    }

  } /* mrn */

} /* bwi_guidance */
