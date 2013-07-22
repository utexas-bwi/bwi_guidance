#ifndef SIMPLE_MODEL_3AJSZE2C
#define SIMPLE_MODEL_3AJSZE2C

namespace bwi_exp1 {

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

      typedef uint32_t state_t;
      typedef uint32_t action_t;

      state_t canonicalizeState(uint32_t graph_id, uint32_t direction, uint32_t robots_remaining) const;
      state_t canonicalizeState(const std::vector<float> &state) const;
      action_t canonicalizeAction(int action) const;

      void initializeStateSpace();
      std::vector<std::vector<State> > state_cache_;

      void initializeActionCache();
      void constructActionsAtState(state_t state, std::vector<Action>& actions);
      std::vector<Action>& getActionsAtState(state_t state);
      std::vector<std::vector<Action> > action_cache_;

      void initializeNextStateCache();
      std::vector<std::vector<state_t> > next_state_cache_;
      std::vector<std::vector<std::vector<float> > > ns_distribution_cache_;

      uint32_t num_vertices_;
      uint32_t num_directions_;
      uint32_t max_robots_;
      size_t getStateSpaceSize() const;

      topological_mapper::Graph graph_;

  };
  
} /* bwi_exp1 */


#endif /* end of include guard: SIMPLE_MODEL_3AJSZE2C */
