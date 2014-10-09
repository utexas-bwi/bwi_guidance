#include <boost/foreach.hpp>
#include <cmath>
#include <fstream>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

#include <bwi_guidance_solver/irm/person_model.h>
#include <bwi_mapper/point_utils.h>
#include <bwi_mapper/map_utils.h>

namespace bwi_guidance_solver {

  namespace irm {

    PersonModel::PersonModel(const bwi_mapper::Graph& graph, 
                             const nav_msgs::OccupancyGrid& map,  
                             size_t goal_idx, 
                             const std::string& file, 
                             bool allow_robot_current_idx, 
                             float visibility_range, 
                             bool allow_goal_visibility, 
                             unsigned int max_robots, 
                             float success_reward, 
                             RewardStructure reward_structure, 
                             bool use_importance_sampling) : 
    graph_(graph),
    map_(map), goal_idx_(goal_idx),
    allow_robot_current_idx_(allow_robot_current_idx),
    visibility_range_(visibility_range),
    allow_goal_visibility_(allow_goal_visibility), max_robots_(max_robots),
    success_reward_(success_reward), reward_structure_(reward_structure),
    use_importance_sampling_(use_importance_sampling) {

      // Initialize intrinsic reward cache
      for (size_t i = 0; i < boost::num_vertices(graph_); ++i) {
        std::vector<size_t> temp_path;
        intrinsic_reward_cache_.push_back(bwi_mapper::getShortestPathWithDistance(
                                                                                  i, goal_idx_, temp_path, graph_));
      }

      if (!file.empty()) {
        std::ifstream ifs(file.c_str());
        if (ifs.is_open()) {
          std::cout << "PersonModel: Loading model from file: " << file <<
            std::endl;
          boost::archive::binary_iarchive ia(ifs);
          ia >> *this;
          std::cout << " - Model loaded from file!" << std::endl;
          ifs.close();
          return;
        }
      }

      // Compute Model
      initializeStateSpace();
      initializeActionCache();
      initializeNextStateCache();

      std::cout << "PersonModel: Model Computed!!" << std::endl;

      if (!file.empty()) {
        std::cout << " - Saving to file: " << file <<
          std::endl;
        std::ofstream ofs(file.c_str());
        boost::archive::binary_oarchive oa(ofs);
        oa << *this;
        ofs.close();
      }
    }

    bool PersonModel::isTerminalState(const State& state) const {
      return state.graph_id == goal_idx_;
    }

    void PersonModel::getStateVector(std::vector<State>& states) {
      states = state_cache_;
    }

    void PersonModel::getActionsAtState(const State& state, 
                                             std::vector<Action>& actions) {
      actions = action_cache_[state];
    }

    /** Get the predictions of the MDP model for a given state action */
    void PersonModel::getTransitionDynamics(const State& state, 
                                                 const Action& action, std::vector<State> &next_states, 
                                                 std::vector<float> &rewards, std::vector<float> &probabilities) {

      next_states.clear();
      rewards.clear();
      probabilities.clear();

      if (isTerminalState(state)) {
        return; // no next states for you!
      }

      getNextStates(state, action, next_states); // copy cost
      probabilities = getTransitionProbabilities(state, action);

      rewards.resize(next_states.size());
      // Compute reward based on euclidean distance between state graph ids
      for (std::vector<State>::const_iterator ns = next_states.begin();
           ns != next_states.end(); ++ns) {

        rewards[ns - next_states.begin()] = 0;

        // Add shaping reward as necessary
        if (reward_structure_ == INTRINSIC_REWARD ||
            reward_structure_ == SHAPING_REWARD) {
          rewards[ns - next_states.begin()] +=
            intrinsic_reward_cache_[state.graph_id] - 
            intrinsic_reward_cache_[ns->graph_id];
        }

        if (reward_structure_ == STANDARD_REWARD ||
            reward_structure_ == SHAPING_REWARD) {
          // Standard reward formulation
          rewards[ns - next_states.begin()] +=
            -bwi_mapper::getEuclideanDistance(state.graph_id, ns->graph_id,
                                              graph_);
        }

        if (isTerminalState(*ns)) {
          rewards[ns - next_states.begin()] += success_reward_;
        }

      }
    }

    void PersonModel::takeAction(const State &state, 
                                      const Action &action, float &reward, State &next_state, 
                                      bool &terminal, int& depth_count, boost::shared_ptr<RNG> rng) {

      if (isTerminalState(state)) {
        throw std::runtime_error("Cannot call takeAction() on terminal state");
      }

      std::vector<State> next_states;
      std::vector<float> probabilities;
      std::vector<float> rewards;
      getTransitionDynamics(state, action, next_states, rewards, probabilities); 

      int idx = select(probabilities, rng);
      next_state = next_states[idx];
      reward = rewards[idx];
      terminal = isTerminalState(next_state);
      depth_count = 1;

    }

    void PersonModel::getFirstAction(const State &state, 
                                          Action &action) {
      std::vector<Action>& actions = getActionsAtState(state);
      action = actions[0];
    }

    bool PersonModel::getNextAction(const State &state, 
                                         Action &action) {
      std::vector<Action>& actions = getActionsAtState(state);
      for (size_t i = 0; i < actions.size() - 1; ++i) {
        if (actions[i] == action) {
          action = actions[i + 1];
          return true;
        }
      }
      return false;
    }

    void PersonModel::getAllActions(const State &state,
                                         std::vector<Action>& actions) {
      actions = getActionsAtState(state);
    }

    float PersonModel::getTransitionProbability(const State& state,
                                                     const Action& action, const State& next_state) {
      std::vector<float>& probabilities = 
        getTransitionProbabilities(state, action);
      std::vector<State> next_states;
      getNextStates(state, action, next_states);
      for (unsigned int i = 0; i < next_states.size(); ++i) {
        if (next_states[i] == next_state)
          return probabilities[i];
      }
      return 0;
    }

    void PersonModel::updateRewardStructure(float success_reward, 
                                                 RewardStructure reward_structure, bool use_importance_sampling) {
      success_reward_ = success_reward;
      reward_structure_ = reward_structure;
      use_importance_sampling_ = use_importance_sampling;
    }

    void PersonModel::initializeStateSpace() {

      num_vertices_ = boost::num_vertices(graph_);

      computeAdjacentVertices(adjacent_vertices_map_, graph_);
      computeVisibleVertices(visible_vertices_map_, graph_, map_,
                             visibility_range_);

      state_cache_.clear();
      for (int graph_id = 0; graph_id < num_vertices_; ++graph_id) {
        std::vector<int>& adjacent_vertices = adjacent_vertices_map_[graph_id];
        std::vector<int>& visible_vertices = visible_vertices_map_[graph_id];
        for (int direction = 0; direction < NUM_DIRECTIONS; ++direction) {
          for (int robots = 0; robots <= max_robots_; ++robots) {

            State state;
            state.graph_id = graph_id;
            state.direction = direction;
            state.num_robots_left = robots;

            // If all 5 robots are available, then no other robots can be placed
            if (robots == max_robots_) {
              state.robot_direction = NONE;
              state.visible_robot = NONE;
              state_cache_.push_back(state);
              continue;
            }

            // Otherwise, place all other robots
            for (int rd = DIR_UNASSIGNED; rd < (int)adjacent_vertices.size();
                 ++rd) {

              if (rd == DIR_UNASSIGNED || rd == NONE) { // insert as is
                state.robot_direction = rd; 
              } else { // resolve to adjacent node
                state.robot_direction = adjacent_vertices[rd];
              }

              for (int vr = NONE; vr < (int)visible_vertices.size(); ++vr) {
                // Don't add current_node as an option to visible_robot
                if (vr >= 0 && visible_vertices[vr] == graph_id) {
                  continue; 
                }
                if (vr == NONE) { // insert as is
                  state.visible_robot = vr;
                } else { // resolve to visible node
                  state.visible_robot = visible_vertices[vr];
                }
                state_cache_.push_back(state);
              }
            }
          }
        }
      }
    }

    void PersonModel::initializeActionCache() {
      action_cache_.clear();
      for (std::vector<State>::iterator it = state_cache_.begin(); 
           it != state_cache_.end(); ++it) {
        constructActionsAtState(*it, action_cache_[*it]);
      }
    }

    void PersonModel::constructActionsAtState(const State& state, 
                                                   std::vector<Action>& actions) {

      actions.clear();

      // If a direction has to be assigned in the current state, only one of many
      // DIRECT_PERSON actions can be taken
      if (state.robot_direction == DIR_UNASSIGNED) {
        BOOST_FOREACH(int id, adjacent_vertices_map_[state.graph_id]) {
          actions.push_back(Action(DIRECT_PERSON, id));
        }
        return;
      }

      // Otherwise have the DO_NOTHING option
      actions.push_back(Action(DO_NOTHING,0));

      // Check if the system can place robots
      if (state.num_robots_left != 0) {
        BOOST_FOREACH(int id, visible_vertices_map_[state.graph_id]) {
          if (state.graph_id != id) {
            if (state.visible_robot == NONE) {
              actions.push_back(Action(PLACE_ROBOT, id)); 
            }
          } else {
            if (allow_robot_current_idx_ && state.robot_direction == NONE) {
              actions.push_back(Action(PLACE_ROBOT, id)); 
            }
          }
        }
      }

    }

    std::vector<Action>& PersonModel::getActionsAtState(
                                                                  const State& state) {
      return action_cache_[state];
    }

    void PersonModel::initializeNextStateCache() {

      ns_distribution_cache_.clear();
      BOOST_FOREACH(const State& state, state_cache_) {
        std::vector<Action>& actions = getActionsAtState(state);
        BOOST_FOREACH(const Action& action, actions) {
          constructTransitionProbabilities(state, action, 
                                           ns_distribution_cache_[state][action]);
        }
      }

    }

    void PersonModel::getNextStates(const State& state, const Action& action, 
                                         std::vector<State>& next_states) {

      next_states.clear();

      // getNextStates does not check if this action was indeed allowed at the
      // given state. With an incorrect action, this function will probably lead
      // you to a non existent state.

      if (isTerminalState(state)) {
        return; // no next states
      }

      // First figure out next states for all actions that will end up in a
      // deterministic state transition
      if (action.type == PLACE_ROBOT) {
        State next_state = state;
        next_state.num_robots_left--;
        if (action.graph_id != state.graph_id) {
          next_state.visible_robot = action.graph_id;
          next_states.push_back(next_state);
          return;
        } else {
          next_state.robot_direction = DIR_UNASSIGNED;
          next_states.push_back(next_state);
          return;
        }
      }

      if (action.type == DIRECT_PERSON) {
        State next_state = state;
        next_state.robot_direction = action.graph_id;
        next_states.push_back(next_state);
        return;
      }

      // Implies action.type == DO_NOTHING. The next state will not be
      // deterministic. Compute all possible next states. Each next state graph
      // id willl be a member of adjacent nodes, and the direction will be
      // computed automatically

      // Get all adjacent ids the person can transition to
      // Algorithm 1 in paper
      next_states.resize(adjacent_vertices_map_[state.graph_id].size());
      unsigned int count = 0;
      BOOST_FOREACH(int next_node, adjacent_vertices_map_[state.graph_id]) {
        State &next_state = next_states[count];
        if (state.visible_robot == NONE) {
          // If no robot was visible in previous state, no robot can be present
          next_state.robot_direction = NONE;
          next_state.visible_robot = NONE;
        } else {
          if (state.visible_robot == next_node) {
            // We moved up to a robot, setup a robot here without an assigned dir
            next_state.robot_direction = DIR_UNASSIGNED;
            next_state.visible_robot = NONE; // no longer tracked 
          } else if (std::find(visible_vertices_map_[next_node].begin(),
                               visible_vertices_map_[next_node].end(), state.visible_robot) ==
                     visible_vertices_map_[next_node].end()) { 
            // The person moved such that a previously visible robot is no 
            // longer visible. Decomission the robot.
            next_state.robot_direction = NONE;
            next_state.visible_robot = NONE;
          } else {
            // The case where the tracked robot is still visible
            next_state.robot_direction = NONE;
            next_state.visible_robot = state.visible_robot;
          }
        }

        next_state.direction = computeNextDirection(state.direction,
                                                    state.graph_id, next_node, graph_);
        next_state.num_robots_left = state.num_robots_left;
        next_state.graph_id = next_node;
        ++count;
      }
    }

    void PersonModel::constructTransitionProbabilities(const State& state, 
                                                            const Action& action, std::vector<float>& probabilities) {

      probabilities.clear();

      // constructTransitionProbabilities does not check if this action was
      // indeed allowed at the given state. With an incorrect action, this
      // function will probably lead you to a non existent state.

      if (isTerminalState(state)) {
        return; // since next_states.size == 0
      }

      if (action.type == DIRECT_PERSON || action.type == PLACE_ROBOT) {
        probabilities.push_back(1.0f); //since next_states.size == 1
        return;
      }

      // If the person is going to make the transition here (i.e action = DO_NOTHING),
      // then use the hand-coded human motion model for producing the transition
      // function

      // In the future this human motion model needs to be abstracted to an 
      // independent data structure so that it can be learned given a state 
      // feature vector.

      /* CASE 1 */

      // In case the next robot is somewhere in the direction of where the person
      // wants to go, this should significantly increase the probablity of seeing
      // the next robot and moving towards it.

      std::vector<int>& visible_vertices = 
        visible_vertices_map_[state.graph_id];
      bool goal_visible = allow_goal_visibility_ &&
        std::find(visible_vertices.begin(), visible_vertices.end(), goal_idx_) !=
        visible_vertices.end();
      bool case_1_invalid = true;
      if (state.visible_robot != NONE || goal_visible) {
        int target = (goal_visible) ? goal_idx_ : state.visible_robot;
        // Check angle to target
        float expected_dir = 
          bwi_mapper::getNodeAngle(state.graph_id, target, graph_);
        float angle_diff = getAbsoluteAngleDifference(expected_dir, 
                                                      getAngleInRadians(state.direction));
        if (angle_diff < M_PI / 3) {
          // target is inside visibility cone, so we are pretty sure it has been
          // seen. case 1 is finally true!
          case_1_invalid = false;

          std::vector<State> next_states;
          getNextStates(state, action, next_states);

          std::vector<float> differences;
          BOOST_FOREACH(const State& next_state, next_states) {
            float ns_angle = bwi_mapper::getNodeAngle(state.graph_id,
                                                      next_state.graph_id, graph_);
            float ns_difference = getAbsoluteAngleDifference(expected_dir,
                                                             ns_angle);
            differences.push_back(ns_difference);
          }
          unsigned best_ns = std::distance(differences.begin(),
                                           std::min_element(differences.begin(), differences.end()));

          unsigned robot_dir = (state.robot_direction != NONE) ? 
            state.robot_direction : next_states[best_ns].graph_id;

          unsigned next_state_counter = 0;
          float probablity_sum = 0.0f;
          BOOST_FOREACH(const State& next_state, next_states) {
            float probablity = 0.01f / next_states.size();
            if (best_ns == next_state_counter)
              probablity += 0.495f;
            if (robot_dir == next_state.graph_id)
              probablity += 0.495f;
            probablity_sum += probablity;
            if (next_state_counter == next_states.size() - 1) {
              // Account for floating point errors. No surprises!
              probablity += 1.0f - probablity_sum; 
            }
            probabilities.push_back(probablity);
            next_state_counter++;
          }
        }
      }

      if (!case_1_invalid) {
        return;
      }

      /* Case 2 and 3 */
      float expected_dir;
      if (state.robot_direction != NONE) {
        // Case 2
        if (state.robot_direction == DIR_UNASSIGNED) {
          // The DO_NOTHING action should not be possible in this state
          throw std::runtime_error("Human Model: unassigned robot_dir!!!");
        }
        expected_dir = bwi_mapper::getNodeAngle(state.graph_id,
                                                state.robot_direction, graph_);
      } else {
        expected_dir = getAngleInRadians(state.direction);
      }

      // Now compute the weight of each next state. Get the favored direction
      // and compute transition probabilities
      std::vector<State> next_states;
      getNextStates(state, action, next_states);

      float weight_sum = 0;
      std::vector<float> weights;
      BOOST_FOREACH(const State& next_state, next_states) {

        float next_state_direction = bwi_mapper::getNodeAngle(
                                                              state.graph_id, next_state.graph_id, graph_);
        float angle_difference = getAbsoluteAngleDifference(next_state_direction, 
                                                            expected_dir);

        // Compute the probability of this state
        float weight = exp(-pow(angle_difference, 2) / (2 * 0.1));
        weights.push_back(weight);
        weight_sum += weight;
      }

      // Normalize probabilities. Ensure sum == 1 with last non zero probability 
      float probability_sum = 0;
      for (size_t probability_counter = 0; probability_counter < weights.size();
           ++probability_counter) {
        float probability = 0.9 * (weights[probability_counter] / weight_sum) +
          0.1 * (1.0f / weights.size());
        probability_sum += probability;
        if (probability_counter == weights.size() - 1) {
          // Account for floating point errors. No surprises!
          probability += 1.0f - probability_sum; 
        }
        probabilities.push_back(probability);
      }
    }

    std::vector<float>& PersonModel::getTransitionProbabilities(const State& state, const Action& action) {
      return ns_distribution_cache_[state][action];
    }

  } /* irm - InstantaneousRobotMotion */

} /* bwi_guidance_solver */
