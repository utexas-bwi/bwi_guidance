#ifndef BWI_GUIDANCE_SOLVER_PERSON_MODEL_IROS14
#define BWI_GUIDANCE_SOLVER_PERSON_MODEL_IROS14

#include <nav_msgs/OccupancyGrid.h>
#include <rl_pursuit/planning/Model.h>
#include <stdint.h>

#include <bwi_guidance_solver/structures_qrr14.h>
#include <bwi_guidance_solver/utils.h>
#include <bwi_mapper/graph.h>

namespace boost {
  namespace serialization {
    class access;
  }
}

namespace bwi_guidance {

  enum RewardStructureIROS14 {
    STANDARD_REWARD = 0,
    SHAPING_REWARD = 1
  };

  class PersonModelIROS14 : public Model<StateIROS14, ActionIROS14> {

    public:

      PersonModelIROS14(const bwi_mapper::Graph& graph, 
          const nav_msgs::OccupancyGrid& map, size_t goal_idx, 
          const std::string& file = "", float visibility_range = 0.0f, 
          bool allow_goal_visibility = false, float success_reward = 0.0f,
          RewardStructure reward_structure = STANDARD_REWARD,);

      /* Functions inherited from PredictiveModel */
      virtual bool isTerminalState(const StateIROS14& state) const;
      virtual void getTransitionDynamics(const StateIROS14 &s, 
          const ActionIROS14 &a, std::vector<StateIROS14> &next_states, 
          std::vector<float> &rewards, std::vector<float> &probabilities);

      virtual ~PersonModelIROS14() {};

      virtual std::string generateDescription(unsigned int indentation = 0) {
        return std::string("stub");
      }

      /* Functions inherited from Model */
      virtual void setState(const StateIROS14 &state);
      virtual void takeAction(const ActionIROS14 &action, float &reward, 
          StateIROS14 &state, bool &terminal);
      virtual void getFirstAction(const StateIROS14 &state, ActionIROS14 &action);
      virtual bool getNextAction(const StateIROS14 &state, ActionIROS14 &action);
      virtual float getTransitionProbability(const StateIROS14& state, 
          const ActionIROS14& action, const StateIROS14& next_state);

      void initializeRNG(URGenPtr generator);
      void updateRewardStructure(float success_reward, RewardStructure
          reward_structure, bool use_importance_sampling);

      void getNextStates(const StateIROS14& state, const ActionIROS14& action, 
          std::vector<StateIROS14>& next_states);

    private:

      /* Current state for generative model */
      StateIROS14 current_state_;
      URGenPtr generator_;
      std::vector<float> intrinsic_reward_cache_;
      RewardStructure reward_structure_;
      float success_reward_;
      bool use_importance_sampling_;

      /* StateIROS14 space cache */
      std::map<int, std::vector<int> > adjacent_vertices_map_;
      std::map<int, std::vector<int> > visible_vertices_map_;
      void computeAdjacentVertices();
      void computeVisibleVertices();

      /* Actions cache */
      void initializeActionCache();
      void constructActionsAtState(const StateIROS14& state, 
          std::vector<ActionIROS14>& actions);
      std::vector<ActionIROS14>& getActionsAtState(const StateIROS14 &state);
      std::map<StateIROS14, std::vector<ActionIROS14> > action_cache_;

      /* Next states and transitions cache */
      void initializeNextStateCache();
      std::map<StateIROS14, std::map<ActionIROS14, std::vector<float> > > 
        ns_distribution_cache_;
      void constructTransitionProbabilities(const StateIROS14& state, 
          const ActionIROS14& action, std::vector<float>& probabilities);
      std::vector<float>& getTransitionProbabilities(const StateIROS14& state,
          const ActionIROS14& action);

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
  
} /* bwi_guidance */

BOOST_CLASS_TRACKING(bwi_guidance::PersonModelIROS14, 
    boost::serialization::track_never)

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_PERSON_MODEL_IROS14 */
