#ifndef SIMPLE_MODEL_3AJSZE2C
#define SIMPLE_MODEL_3AJSZE2C

#include <stdint.h>

#include <bwi_exp1_solver/PredictiveModel.h>
#include <bwi_exp1_solver/structures.h>
#include <topological_mapper/graph.h>

namespace bwi_exp1 {

  class PersonModel : public PredictiveModel<state_t, action_t> {

    public:

      PersonModel(const topological_mapper::Graph& graph, size_t goal_idx);

      virtual bool isTerminalState(const state_t& state) const;
      virtual void getStateVector(std::vector<state_t>& states);
      virtual void getActionsAtState(const state_t &state, std::vector<action_t>& actions);
      virtual void getTransitionDynamics(const state_t &s, 
          const action_t &a, std::vector<state_t> &next_states, 
          std::vector<float> &rewards, std::vector<float> &probabilities);

      virtual ~PersonModel() {};
      virtual std::string generateDescription(unsigned int indentation = 0) {
        return std::string("stub");
      }

      size_t getStateSpaceSize() const;
      state_t canonicalizeState(uint32_t graph_id, uint32_t direction, uint32_t robots_remaining) const;
      State resolveState(state_t state);
      Action resolveAction(state_t state, action_t action);

    private:


      void initializeStateSpace();
      std::vector<State> state_cache_;

      void initializeActionCache();
      void constructActionsAtState(state_t state, std::vector<Action>& actions);
      std::vector<Action>& getActionsAtState(const state_t &state);
      std::vector<std::vector<Action> > action_cache_;

      void initializeNextStateCache();
      void constructNextStatesAtState(state_t state, 
          std::vector<state_t>& next_states);
      std::vector<state_t>& getNextStatesAtState(state_t state);
      void constructTransitionProbabilities(state_t state_id, action_t action_id, 
        std::vector<float>& probabilities);
      std::vector<float>& getTransitionProbabilities(state_t state_id, action_t action_id);
      std::vector<std::vector<state_t> > next_state_cache_;
      std::vector<std::vector<std::vector<float> > > ns_distribution_cache_;

      size_t computeNextDirection(size_t dir, size_t graph_id, size_t next_graph_id);
      float getAngleFromStates(size_t graph_id, size_t next_graph_id);
      float getDistanceFromStates(size_t graph_id, size_t next_graph_id);
      size_t getDirectionFromAngle(float angle);
      float getAngleFromDirection(size_t dir);
      uint32_t num_vertices_;
      uint32_t num_directions_;
      uint32_t max_robots_;

      topological_mapper::Graph graph_;
      size_t goal_idx_;

  };
  
} /* bwi_exp1 */

#endif /* end of include guard: SIMPLE_MODEL_3AJSZE2C */
