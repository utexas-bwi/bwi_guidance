#ifndef PERSON_MODEL_2
#define PERSON_MODEL_2

#include <stdint.h>

#include <nav_msgs/OccupancyGrid.h>
#include <bwi_exp1_solver/PredictiveModel.h>
#include <bwi_exp1_solver/structures.h>
#include <topological_mapper/graph.h>

namespace boost {
  namespace serialization {
    class access;
  }
}

namespace bwi_exp1 {

  class PersonModel2 : public PredictiveModel<State2, Action> {

    public:

      PersonModel2(const topological_mapper::Graph& graph, 
          const nav_msgs::OccupancyGrid& map, size_t goal_idx, 
          const std::string& file = "", bool allow_robot_current_idx = false);

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

      void getNextStates(const State2& state, const Action& action, 
          std::vector<State2>& next_states);

    private:

      void computeAdjacentVertices();
      void computeVisibleVertices();
      void initializeStateSpace();
      std::map<int, std::vector<int> > adjacent_vertices_map_;
      std::map<int, std::vector<int> > visible_vertices_map_;
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
      uint32_t max_robots_;

      friend class boost::serialization::access;
      template<class Archive>
      void serialize(Archive & ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(adjacent_vertices_map_);
        ar & BOOST_SERIALIZATION_NVP(visible_vertices_map_);
        ar & BOOST_SERIALIZATION_NVP(state_cache_);
        ar & BOOST_SERIALIZATION_NVP(action_cache_);
        ar & BOOST_SERIALIZATION_NVP(next_state_cache_);
        ar & BOOST_SERIALIZATION_NVP(ns_distribution_cache_);
        ar & num_vertices_;
        ar & max_robots_;
      }

      topological_mapper::Graph graph_;
      nav_msgs::OccupancyGrid map_;
      size_t goal_idx_;
      bool allow_robot_current_idx_;

  };
  
} /* bwi_exp1 */

BOOST_CLASS_TRACKING(bwi_exp1::PersonModel2, boost::serialization::track_never)

#endif /* end of include guard: PERSON_MODEL_2 */
