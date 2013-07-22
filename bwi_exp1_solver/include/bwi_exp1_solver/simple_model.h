#ifndef SIMPLE_MODEL_3AJSZE2C
#define SIMPLE_MODEL_3AJSZE2C

#include <stdint.h>

#include <rl_common/core.hh>
#include <topological_mapper/graph.h>

namespace bwi_exp1 {

  struct State {
    size_t graph_id; // ~100
    size_t direction; // 0 to NUM_DIRECTIONS - 1
    size_t num_robots_left; // 0 to MAX_ROBOTS
  };

  enum ActionType {
    DO_NOTHING = 0,
    PLACE_ROBOT = 1
  };

  class Action {
    public:
      Action();
      Action(ActionType a, size_t g);
      ActionType type;
      size_t graph_id; // with PLACE ROBOT, identifies the direction pointed to
  };

  typedef uint32_t state_t;
  typedef uint32_t action_t;

  class PersonModel : public MDPModel {

    public:

      PersonModel(const topological_mapper::Graph& graph, size_t goal_idx);

      /** Update the MDP model with a vector of experiences. */
      virtual bool updateWithExperiences(std::vector<experience> &instances) = 0;

      /** Update the MDP model with a single experience. */
      virtual bool updateWithExperience(experience &instance) = 0;

      /** Get the predictions of the MDP model for a given state action */
      virtual float getStateActionInfo(const std::vector<float> &state, int action, StateActionInfo* retval) = 0;

      /** Get a copy of the MDP Model */
      virtual PersonModel* getCopy() = 0;
      virtual ~PersonModel() {};

    private:

      state_t canonicalizeState(uint32_t graph_id, uint32_t direction, uint32_t robots_remaining) const;
      state_t canonicalizeState(const std::vector<float> &state) const;
      action_t canonicalizeAction(int action) const;
      void produceContinuousState(state_t state_id, std::vector<float>& state);

      void initializeStateSpace();
      std::vector<State> state_cache_;

      void initializeActionCache();
      void constructActionsAtState(state_t state, std::vector<Action>& actions);
      std::vector<Action>& getActionsAtState(state_t state);
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
      size_t getStateSpaceSize() const;

      topological_mapper::Graph graph_;
      size_t goal_idx_;

  };
  
} /* bwi_exp1 */

#endif /* end of include guard: SIMPLE_MODEL_3AJSZE2C */
