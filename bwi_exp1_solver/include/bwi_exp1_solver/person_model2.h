#ifndef PERSON_MODEL_2
#define PERSON_MODEL_2

#include <stdint.h>

#include <nav_msgs/OccupancyGrid.h>
#include <bwi_exp1_solver/PredictiveModel.h>
#include <bwi_exp1_solver/structures.h>
#include <bwi_mapper/graph.h>

namespace boost {
  namespace serialization {
    class access;
  }
}

namespace bwi_exp1 {

  class PersonModel2 : public PredictiveModel<State, Action> {

    public:

      PersonModel2(const bwi_mapper::Graph& graph, 
          const nav_msgs::OccupancyGrid& map, size_t goal_idx, 
          const std::string& file = "", bool allow_robot_current_idx = false,
          float visibility_range = 0.0f, bool allow_goal_visibility = false,
          unsigned int max_robots = 5);

      virtual bool isTerminalState(const State& state) const;
      virtual void getStateVector(std::vector<State>& states);
      virtual void getActionsAtState(const State &state, 
          std::vector<Action>& actions);
      virtual void getTransitionDynamics(const State &s, 
          const Action &a, std::vector<State> &next_states, 
          std::vector<float> &rewards, std::vector<float> &probabilities);

      virtual ~PersonModel2() {};
      virtual std::string generateDescription(unsigned int indentation = 0) {
        return std::string("stub");
      }

      void getNextStates(const State& state, const Action& action, 
          std::vector<State>& next_states);

    private:

      /* State space cache */
      std::map<int, std::vector<int> > adjacent_vertices_map_;
      std::map<int, std::vector<int> > visible_vertices_map_;
      void computeAdjacentVertices();
      void computeVisibleVertices();
      std::vector<State> state_cache_;
      void initializeStateSpace();

      /* Actions cache */
      void initializeActionCache();
      void constructActionsAtState(const State& state, 
          std::vector<Action>& actions);
      std::vector<Action>& getActionsAtState(const State &state);
      std::map<State, std::vector<Action> > action_cache_;

      /* Next states and transitions cache */
      void initializeNextStateCache();
      std::map<State, std::map<Action, std::vector<float> > > 
        ns_distribution_cache_;
      void constructTransitionProbabilities(const State& state, 
          const Action& action, std::vector<float>& probabilities);
      std::vector<float>& getTransitionProbabilities(const State& state,
          const Action& action);

      unsigned int num_vertices_;
      unsigned int max_robots_;

      friend class boost::serialization::access;
      template<class Archive>
      void serialize(Archive & ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(adjacent_vertices_map_);
        ar & BOOST_SERIALIZATION_NVP(visible_vertices_map_);
        ar & BOOST_SERIALIZATION_NVP(state_cache_);
        ar & BOOST_SERIALIZATION_NVP(action_cache_);
        ar & BOOST_SERIALIZATION_NVP(ns_distribution_cache_);
        ar & num_vertices_;
      }

      bwi_mapper::Graph graph_;
      nav_msgs::OccupancyGrid map_;
      size_t goal_idx_;

      /* Some parameters different between exp1 and exp2 */
      bool allow_robot_current_idx_;
      bool allow_goal_visibility_;
      float visibility_range_;

  };
  
} /* bwi_exp1 */

BOOST_CLASS_TRACKING(bwi_exp1::PersonModel2, boost::serialization::track_never)

#endif /* end of include guard: PERSON_MODEL_2 */
