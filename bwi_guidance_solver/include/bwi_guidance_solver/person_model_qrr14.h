#ifndef BWI_GUIDANCE_SOLVER_PERSON_MODEL_QRR14
#define BWI_GUIDANCE_SOLVER_PERSON_MODEL_QRR14

#include <nav_msgs/OccupancyGrid.h>
#include <bwi_rl/planning/Model.h>
#include <bwi_rl/planning/PredictiveModel.h>
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

  enum RewardStructure {
    STANDARD_REWARD = 0,
    INTRINSIC_REWARD = 1,
    SHAPING_REWARD = 2
  };

  class PersonModelQRR14 : public PredictiveModel<StateQRR14, ActionQRR14>,
                           public Model<StateQRR14, ActionQRR14> {

    public:

      PersonModelQRR14(const bwi_mapper::Graph& graph, 
          const nav_msgs::OccupancyGrid& map, size_t goal_idx, 
          const std::string& file = "", bool allow_robot_current_idx = false,
          float visibility_range = 0.0f, bool allow_goal_visibility = false,
          unsigned int max_robots = 5, float success_reward = 0.0f,
          RewardStructure reward_structure = STANDARD_REWARD,
          bool use_importance_sampling = false);

      /* Functions inherited from PredictiveModel */
      virtual bool isTerminalState(const StateQRR14& state) const;
      virtual void getStateVector(std::vector<StateQRR14>& states);
      virtual void getActionsAtState(const StateQRR14 &state, 
          std::vector<ActionQRR14>& actions);
      virtual void getTransitionDynamics(const StateQRR14 &s, 
          const ActionQRR14 &a, std::vector<StateQRR14> &next_states, 
          std::vector<float> &rewards, std::vector<float> &probabilities);

      virtual ~PersonModelQRR14() {};

      virtual std::string generateDescription(unsigned int indentation = 0) {
        return std::string("stub");
      }

      /* Functions inherited from Model */
      virtual void takeAction(const StateQRR14 &state, const ActionQRR14 &action, float &reward, 
          StateQRR14 &next_state, bool &terminal, int &depth_count, boost::shared_ptr<RNG> rng);
      virtual void getFirstAction(const StateQRR14 &state, ActionQRR14 &action);
      virtual bool getNextAction(const StateQRR14 &state, ActionQRR14 &action);
      virtual void getAllActions(const StateQRR14 &state, std::vector<ActionQRR14>& actions);
      virtual float getTransitionProbability(const StateQRR14& state, 
          const ActionQRR14& action, const StateQRR14& next_state);

      void updateRewardStructure(float success_reward, RewardStructure
          reward_structure, bool use_importance_sampling);

      void getNextStates(const StateQRR14& state, const ActionQRR14& action, 
          std::vector<StateQRR14>& next_states);

    private:

      std::vector<float> intrinsic_reward_cache_;
      RewardStructure reward_structure_;
      float success_reward_;
      bool use_importance_sampling_;

      /* StateQRR14 space cache */
      std::map<int, std::vector<int> > adjacent_vertices_map_;
      std::map<int, std::vector<int> > visible_vertices_map_;
      std::vector<StateQRR14> state_cache_;
      void initializeStateSpace();

      /* Actions cache */
      void initializeActionCache();
      void constructActionsAtState(const StateQRR14& state, 
          std::vector<ActionQRR14>& actions);
      std::vector<ActionQRR14>& getActionsAtState(const StateQRR14 &state);
      std::map<StateQRR14, std::vector<ActionQRR14> > action_cache_;

      /* Next states and transitions cache */
      void initializeNextStateCache();
      std::map<StateQRR14, std::map<ActionQRR14, std::vector<float> > > 
        ns_distribution_cache_;
      void constructTransitionProbabilities(const StateQRR14& state, 
          const ActionQRR14& action, std::vector<float>& probabilities);
      std::vector<float>& getTransitionProbabilities(const StateQRR14& state,
          const ActionQRR14& action);

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

BOOST_CLASS_TRACKING(bwi_guidance::PersonModelQRR14, 
    boost::serialization::track_never)

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_PERSON_MODEL_QRR14 */
