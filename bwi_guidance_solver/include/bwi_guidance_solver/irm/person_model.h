#ifndef BWI_GUIDANCE_SOLVER_IRM_PERSON_MODEL_H
#define BWI_GUIDANCE_SOLVER_IRM_PERSON_MODEL_H

#include <nav_msgs/OccupancyGrid.h>
#include <bwi_rl/planning/Model.h>
#include <bwi_rl/planning/PredictiveModel.h>
#include <stdint.h>

#include <bwi_guidance_solver/irm/structures.h>
#include <bwi_guidance_solver/utils.h>
#include <bwi_mapper/graph.h>

#define DEFAULT_MAX_ROBOTS 5

namespace boost {
  namespace serialization {
    class access;
  }
}

namespace bwi_guidance_solver {

  namespace irm {
  
    enum RewardStructure {
      STANDARD_REWARD = 0,
      INTRINSIC_REWARD = 1,
      SHAPING_REWARD = 2
    };

    class PersonModel : public PredictiveModel<State, Action>, public Model<State, Action> {

      public:

        PersonModel(const bwi_mapper::Graph& graph, 
            const nav_msgs::OccupancyGrid& map, size_t goal_idx, 
            const std::string& file = "", bool allow_robot_current_idx = false,
            float visibility_range = 0.0f, bool allow_goal_visibility = false,
            unsigned int max_robots = DEFAULT_MAX_ROBOTS, float success_reward = 0.0f,
            RewardStructure reward_structure = STANDARD_REWARD);

        /* Functions inherited from PredictiveModel */
        virtual bool isTerminalState(const State& state) const;
        virtual void getStateVector(std::vector<State>& states);
        virtual void getActionsAtState(const State &state, 
            std::vector<Action>& actions);
        virtual void getTransitionDynamics(const State &s, 
            const Action &a, std::vector<State> &next_states, 
            std::vector<float> &rewards, std::vector<float> &probabilities);

        virtual ~PersonModel() {};

        virtual std::string generateDescription(unsigned int indentation = 0) {
          return std::string("stub");
        }

        /* Functions inherited from Model */
        virtual void takeAction(const State &state, const Action &action, float &reward, 
            State &next_state, bool &terminal, int &depth_count, boost::shared_ptr<RNG> rng);
        virtual void getFirstAction(const State &state, Action &action);
        virtual bool getNextAction(const State &state, Action &action);
        virtual void getAllActions(const State &state, std::vector<Action>& actions);
        virtual float getTransitionProbability(const State& state, 
            const Action& action, const State& next_state);

        void updateRewardStructure(float success_reward, RewardStructure reward_structure);

        void getNextStates(const State& state, const Action& action, 
            std::vector<State>& next_states);

      private:

        std::vector<float> intrinsic_reward_cache_;
        RewardStructure reward_structure_;
        float success_reward_;

        /* State space cache */
        std::map<int, std::vector<int> > adjacent_vertices_map_;
        std::map<int, std::vector<int> > visible_vertices_map_;
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
  
  } /* irm - InstantaneousRobotMotion */

} /* bwi_guidance_solver */

BOOST_CLASS_TRACKING(bwi_guidance_solver::irm::PersonModel, boost::serialization::track_never)

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_IRM_PERSON_MODEL_H */
