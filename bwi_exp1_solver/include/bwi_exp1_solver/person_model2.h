#ifndef PERSON_MODEL_2
#define PERSON_MODEL_2

#include <stdint.h>

#include <nav_msgs/OccupancyGrid.h>
#include <bwi_exp1_solver/PredictiveModel.h>
#include <bwi_exp1_solver/structures.h>
#include <topological_mapper/graph.h>

namespace bwi_exp1 {

  class PersonModel2 : public PredictiveModel<State2, Action> {

    public:

      PersonModel2(const topological_mapper::Graph& graph, 
          const nav_msgs::OccupancyGrid& map, size_t goal_idx);

      virtual bool isTerminalState(const State2& state) const;
      virtual void getStateVector(std::vector<State2>& states);
      virtual void getActionsAtState(const State2 &state, 
          std::vector<Action>& actions);
      virtual void getTransitionDynamics(const State2 &s, 
          const Action &a, std::vector<State2> &next_states, 
          std::vector<float> &rewards, std::vector<float> &probabilities);

      virtual ~PersonModel2() {};
      virtual std::string generateDescription(unsigned int indentation = 0) {
        return std::string("stub");
      }

      size_t computeNextDirection(size_t dir, 
          size_t graph_id, size_t next_graph_id);
      size_t getDirectionFromAngle(float angle);
      float getAngleFromDirection(size_t dir);
      float getAngleFromStates(size_t graph_id, size_t next_graph_id);
      float getDistanceFromStates(size_t graph_id, size_t next_graph_id);
      void getNextStates(const State2& state, const Action& action, 
          std::vector<State2>& next_states);

    private:

      void computeAdjacentVertices();
      void computeRobotVertices();
      void initializeStateSpace();
      std::map<int, std::vector<int> > adjacent_vertices_map_;
      std::map<int, std::vector<int> > robot_vertices_map_;
      std::vector<State2> state_cache_;

      void initializeActionCache();
      void constructActionsAtState(const State2& state, 
          std::vector<Action>& actions);
      std::vector<Action>& getActionsAtState(const State2 &state);
      std::map<State2, std::vector<Action> > action_cache_;

      void initializeNextStateCache();
      // maps a direction, graph id to all possible next states and graph id
      std::vector<std::vector<std::pair<int, int> > > next_state_cache_; 
      void constructTransitionProbabilities(const State2& state, 
          const Action& action, std::vector<float>& probabilities);
      std::vector<float>& getTransitionProbabilities(const State2& state,
          const Action& action);
      std::map<State2, std::map<Action, std::vector<float> > > 
        ns_distribution_cache_;

      uint32_t num_vertices_;
      uint32_t num_directions_;
      uint32_t max_robots_;

      topological_mapper::Graph graph_;
      nav_msgs::OccupancyGrid map_;
      size_t goal_idx_;

  };
  
} /* bwi_exp1 */

#endif /* end of include guard: PERSON_MODEL_2 */
