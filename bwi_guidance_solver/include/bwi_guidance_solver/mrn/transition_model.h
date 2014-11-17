#ifndef BWI_GUIDANCE_SOLVER_MRN_TRANSITION_MODEL_H
#define BWI_GUIDANCE_SOLVER_MRN_TRANSITION_MODEL_H

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <bwi_guidance_solver/common.h>
#include <bwi_guidance_solver/mrn/common.h>
#include <bwi_guidance_solver/mrn/structures.h>

namespace bwi_guidance_solver {

  namespace mrn {

    class HumanDecisionModel {

      public: 

        typedef boost::shared_ptr<HumanDecisionModel> Ptr;

        HumanDecisionModel(const bwi_mapper::Graph graph, float decision_variance_multiplier = 1.0f) : 
            graph_(graph), decision_variance_multiplier_(decision_variance_multiplier) {
          computeAdjacentVertices(adjacent_vertices_map_, graph_);
        }

        virtual ~HumanDecisionModel() {}

        // TODO write a version that draws the probability on the image as well.
        virtual int GetNextNode(const State &state, RNG &rng) {

          // In case no help has been provided to the human, the expected direction of motion can be given by. 
          float expected_direction_of_motion;
          float expected_variance;

          if (state.assist_type == LEAD_PERSON) {
            // Assume model is deterministic, and follows the system's assistance perfectly.
            return state.assist_loc;
          } else if (state.assist_type == DIRECT_PERSON) {
            expected_variance = 0.05f * decision_variance_multiplier_;
            expected_direction_of_motion = bwi_mapper::getNodeAngle(state.loc_node, state.assist_loc, graph_);
          } else /* no assistance provided. */ {
            // Do nothin. Use default values for exoe
            if (state.loc_prev != state.loc_node) {
              expected_direction_of_motion = bwi_mapper::getNodeAngle(state.loc_prev, state.loc_node, graph_);
              expected_variance = 0.1f * decision_variance_multiplier_;
            } else {
              // This can only mean that this is the start state (as the person does not have loc_prev set), and 
              // the policy called Wait without first calling LEAD or DIRECT, which is a terrible action since
              // some free help went to waste. Assume that the person will move completely randomly.
              int rand_idx = rng.randomInt(adjacent_vertices_map_[state.loc_node].size() - 1);
              return adjacent_vertices_map_[state.loc_node][rand_idx];
            }
          }

          // Now assume that the person moves to one the adjacent locations
          float weight_sum = 0;
          std::vector<float> weights;
          BOOST_FOREACH(int adj, adjacent_vertices_map_[state.loc_node]) {
            float next_state_direction = bwi_mapper::getNodeAngle(state.loc_node, adj, graph_);
            float angle_difference = getAbsoluteAngleDifference(next_state_direction, expected_direction_of_motion);

            // Compute the probability of this state
            float weight = expf(-powf(angle_difference, 2) / (2 * expected_variance));
            weights.push_back(weight);
            weight_sum += weight;
          }

          float probability_sum = 0;
          std::vector<float> probabilities;
          /* std::cout << "Transition probabilities: " << std::endl; */
          for (size_t probability_counter = 0; probability_counter < weights.size(); ++probability_counter) {
            /* std::cout << weights[probability_counter] << " " << weight_sum << std::endl; */
            double probability = 0.99 * (weights[probability_counter] / weight_sum) + 0.01 * (1.0f / weights.size());
            probability_sum += probability;
            if (probability_counter == weights.size() - 1) {
              // Account for floating point errors. No surprises!
              probability += 1.0f - probability_sum; 
            }
            probabilities.push_back(probability);
            // std::cout << "  to " << 
            //   adjacent_vertices_map_[next_state.graph_id][probability_counter] <<
            //   ": " << probability << std::endl;
          }

          return adjacent_vertices_map_[state.loc_node][rng.select(probabilities)];
        }

      private:

        bwi_mapper::Graph graph_;
        std::map<int, std::vector<int> > adjacent_vertices_map_;
        float decision_variance_multiplier_;

    };

    class TaskGenerationModel {

      public:

        typedef boost::shared_ptr<TaskGenerationModel> Ptr;

        TaskGenerationModel(const std::vector<int> robot_home_base,
                            const bwi_mapper::Graph &graph,
                            float task_utility,
                            bool home_base_only = false) : 
          robot_home_base_(robot_home_base), task_utility_(task_utility), home_base_only_(home_base_only) {
          cacheNewGoalsByDistance(graph);
        }

        virtual ~TaskGenerationModel() {}

        virtual void generateNewTaskForRobot(int robot_id, RobotState &robot, RNG &rng) {
          // Optimized!!!
          if (home_base_only_) {
            robot.tau_d = robot_home_base_[robot_id];
          } else {
            int idx = robot_home_base_[robot_id];
            int graph_distance = rng.poissonInt(1);
            while(graph_distance >= goals_by_distance_[idx].size()) {
              graph_distance = rng.poissonInt(1);
            }
            std::vector<int>& possible_goals = goals_by_distance_[idx][graph_distance];
            robot.tau_d = *(possible_goals.begin() + rng.randomInt(possible_goals.size() - 1));
          }
          robot.tau_t = 0.0f;
          robot.tau_total_task_time = 5.0f;
          robot.tau_u = task_utility_;
        }

        void cacheNewGoalsByDistance(const bwi_mapper::Graph &graph) {

          std::map<int, std::vector<int> > adjacent_vertices_map;
          computeAdjacentVertices(adjacent_vertices_map, graph);
          int num_vertices = boost::num_vertices(graph);

          goals_by_distance_.clear();
          goals_by_distance_.resize(num_vertices);
          for (int idx = 0; idx < num_vertices; ++idx) {
            std::set<int> closed_set;
            std::vector<int> current_set;
            current_set.push_back(idx);
            while (current_set.size() != 0) {
              goals_by_distance_[idx].push_back(current_set);
              std::set<int> open_set;
              BOOST_FOREACH(int c, current_set) {
                BOOST_FOREACH(int a, adjacent_vertices_map[c]) {
                  if (std::find(closed_set.begin(), closed_set.end(), a) ==
                      closed_set.end()) {
                    open_set.insert(a);
                  }
                }
              }
              closed_set.insert(current_set.begin(), current_set.end());
              current_set = std::vector<int>(open_set.begin(), open_set.end());
            }
          }
        }

      private:

        std::vector<int> robot_home_base_;
        std::vector<std::vector<std::vector<int> > > goals_by_distance_;

        float task_utility_;
        bool home_base_only_;

    };

    class MotionModel {

      public:

        typedef boost::shared_ptr<MotionModel> Ptr;

        MotionModel(const bwi_mapper::Graph graph, float avg_robot_speed, float avg_human_speed) : 
          graph_(graph), robot_speed_(avg_robot_speed), human_speed_(avg_human_speed) {
          computeShortestPath(shortest_distances_, shortest_paths_, graph_);
        }

        virtual ~MotionModel() {}
        virtual bool move(State &state,
                          int next_node,
                          const TaskGenerationModel::Ptr &task_generation_model,
                          RNG &rng,
                          float &total_time,
                          float time_step = 0.0f) {

          bool ready_for_next_action = false;

          float human_speed = (state.assist_type == LEAD_PERSON) ? robot_speed_ : human_speed_;

          // Move the human.
          if (time_step == 0.0f) {
            time_step = shortest_distances_[state.loc_node][next_node] / human_speed;
            state.loc_prev = state.loc_node;
            state.loc_node = next_node;
            state.loc_p = 0.0f;
            ready_for_next_action = true;
          } else {
            if ((state.loc_prev == next_node) || (state.loc_p == 0.0f)) {
              state.loc_prev = next_node;
              float human_coverable_distance = time_step * human_speed;
              float human_edge_distance = shortest_distances_[state.loc_node][state.loc_prev];
              float added_precision = human_coverable_distance / human_edge_distance;
              state.loc_p += added_precision;
              if (state.loc_p >= 1.0f) {
                // We've reached next node.
                ready_for_next_action = true;
                time_step -= (state.loc_p - 1.0f) * (human_edge_distance / human_speed);
                state.loc_p = 0.0f;
                state.loc_prev = state.loc_node;
                state.loc_node = next_node;
              }
            } else {
              throw std::runtime_error("Human's precise location not well defined for visualization purposes.");
            }
          }

          total_time = time_step;

          // Since the human has moved, remove any previous assistance.
          if (ready_for_next_action) {
            state.assist_type = NONE;
            state.assist_loc = NONE;
          }

          /* std::cout << "Moving ahead for " << time << " seconds" << std::endl; */
          // Optimized!!!
          for (int i = 0; i < state.robots.size(); ++i) {
            RobotState& robot = state.robots[i];

            float robot_time_remaining = time_step;
            bool robot_in_use = robot.help_destination != NONE;
            int destination = (robot_in_use) ? robot.help_destination : robot.tau_d;

            // Get shortest path to destination, and figure out how much distance
            // of that path we can cover
            while (robot_time_remaining > 0.0f) {

              /* std::cout << robot.graph_id << " " << robot.precision << " " << destination << " " << robot.other_graph_node << std::endl; */
              // Check if the robot has already reached it's destination.
              if (isRobotExactlyAt(robot, destination)) {
                if (robot_in_use) {
                  // Won't be doing anything more with this robot until the robot gets released.
                  robot_time_remaining = 0.0f;
                } else {
                  // The robot has reached its service task destination and is in the middle of performing the service
                  // task.
                  robot.tau_t += robot_time_remaining;
                  robot_time_remaining = 0.0f;

                  // Check if the robot can complete this task and move on to the next task.
                  if (robot.tau_t > robot.tau_total_task_time) {
                    robot_time_remaining = robot.tau_t - robot.tau_total_task_time;
                    task_generation_model->generateNewTaskForRobot(i, robot, rng);
                    // Update the destination for this robot.
                    destination = robot.tau_d;
                  } 
                }
              } else {

                // Just in case the robot is exactly at v, let's switch the position around so that the robot is
                // exactly at u.
                if (isRobotExactlyAt(robot, robot.loc_v)) {
                  robot.loc_u = robot.loc_v;
                  robot.loc_p = 0.0f;
                }

                // If the robot is exactly at u, then compute the shortest path to the goal and move the robot along
                // this shortest path.
                if (isRobotExactlyAt(robot, robot.loc_u)) {
                  // Move robot along shortest path from u.
                  std::vector<size_t>& shortest_path = shortest_paths_[robot.loc_u][destination];
                  robot.loc_v = shortest_path[0];
                  robot.loc_p += (robot_time_remaining * robot_speed_) / shortest_distances_[robot.loc_u][robot.loc_v];
                  robot_time_remaining = 0.0f;
                  if (robot.loc_p > 1.0f) {
                    robot_time_remaining = 
                      ((robot.loc_p - 1.0f) * (shortest_distances_[robot.loc_u][robot.loc_v])) / robot_speed_;
                    robot.loc_p = 1.0f;
                  }
                } else {
                  // The robot is somewhere in the middle of u and v.
                  bool shortest_path_through_u = isShortestPathThroughLocU(robot.loc_u,
                                                                           robot.loc_v,
                                                                           robot.loc_p,
                                                                           destination,
                                                                           shortest_distances_);

                  if (shortest_path_through_u) {
                    // Move robot to u.
                    robot.loc_p -= (robot_time_remaining * robot_speed_) / shortest_distances_[robot.loc_u][robot.loc_v];
                    robot_time_remaining = 0.0f;
                    if (robot.loc_p < 0.0f) {
                      robot_time_remaining = 
                        ((-robot.loc_p) * (shortest_distances_[robot.loc_u][robot.loc_v])) / robot_speed_;
                      robot.loc_p = 0.0f;
                    }
                  } else {
                    // Move robot to v.
                    robot.loc_p += (robot_time_remaining * robot_speed_) / shortest_distances_[robot.loc_u][robot.loc_v];
                    robot_time_remaining = 0.0f;
                    if (robot.loc_p > 1.0f) {
                      robot_time_remaining = 
                        ((robot.loc_p - 1.0f) * (shortest_distances_[robot.loc_u][robot.loc_v])) / robot_speed_;
                      robot.loc_p = 1.0f;
                    }
                  }
                }
              }
            }

            // Account for any floating point errors, especially while leading a person.
            if (robot.loc_p > 1.0f - 1e-6) {
              robot.loc_p = 1.0f;
            }

            if (robot.loc_p < 1e-6) {
              robot.loc_p = 0.0f;
            }

          }

          return ready_for_next_action;
        }

        virtual float getHumanSpeed() {
          return human_speed_;
        }

        virtual float getRobotSpeed() {
          return robot_speed_;
        }

      private:

        bwi_mapper::Graph graph_;
        std::vector<std::vector<std::vector<size_t> > > shortest_paths_;
        std::vector<std::vector<float> > shortest_distances_;

        float robot_speed_;
        float human_speed_;

    };

  } /* mrn */

} /* bwi_guidance */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_TRANSITION_MODEL_H */
