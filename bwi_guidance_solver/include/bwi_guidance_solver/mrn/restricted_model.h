#ifndef BWI_GUIDANCE_SOLVER_RESTRICTED_MODEL_H
#define BWI_GUIDANCE_SOLVER_RESTRICTED_MODEL_H

#include <stdint.h>
#include <nav_msgs/OccupancyGrid.h>

#include <bwi_guidance_solver/mrn/person_model.h>
#include <bwi_guidance_solver/mrn/extended_structures.h>
#include <bwi_guidance_solver/mrn/transition_model.h>
#include <bwi_mapper/graph.h>
#include <bwi_rl/planning/Model.h>
#include <bwi_tools/common/Params.h>

namespace bwi_guidance_solver {

  namespace mrn {
    
    class RestrictedModel : public Model<ExtendedState, Action> {

      public:

#define PARAMS(_) \
          _(float,frame_rate,frame_rate,0.0f) \
          _(int,max_assigned_robots,kax_assigned_robots,1)

          Params_STRUCT(PARAMS)
#undef PARAMS

        RestrictedModel(const bwi_mapper::Graph& graph, 
                        const nav_msgs::OccupancyGrid& map, 
                        int goal_idx,
                        const MotionModel::Ptr &motion_model,
                        const HumanDecisionModel::Ptr &human_decision_model,
                        const TaskGenerationModel::Ptr &task_generation_model,
                        const Params &params);

        /* Functions inherited from PredictiveModel */
        virtual ~RestrictedModel() {};

        /* Functions inherited from Model */
        virtual void takeAction(const ExtendedState &state, 
                                const Action &action, 
                                float &reward, 
                                ExtendedState &next_state, 
                                bool &terminal, 
                                int &depth_count, 
                                boost::shared_ptr<RNG> rng);

        /* Same as the inherited takeAction function, but provides more information. */
        void takeAction(const ExtendedState &state, 
                        const Action &action, 
                        float &reward, 
                        ExtendedState &next_state, 
                        bool &terminal, 
                        int &depth_count,
                        boost::shared_ptr<RNG> &rng,
                        float &time_loss,
                        float &utility_loss,
                        std::vector<State> &frame_vector);

        virtual void getFirstAction(const ExtendedState &state, Action &action);
        virtual bool getNextAction(const ExtendedState &state, Action &action);
        virtual void getAllActions(const ExtendedState &state, std::vector<Action>& actions);

        virtual std::string generateDescription(unsigned int indentation = 0) {
          return std::string("stub");
        }

        /* A convenience function to visualize a model. */
        void drawState(const State& state, cv::Mat& image);
     
      private:

        void getActionsAtState(const ExtendedState &state, std::vector<Action>& actions);

        /* Underlying base model. */
        boost::shared_ptr<PersonModel> base_model_;

        /* Some cached data. */
        std::vector<std::vector<std::vector<size_t> > > shortest_paths_;
        std::vector<std::vector<float> > shortest_distances_;
        std::map<int, std::vector<int> > adjacent_vertices_map_;

        std::map<int, std::vector<int> > visible_vertices_map_;
        std::map<int, std::vector<int> > action_vertices_map_;

        Params params_;
        int goal_idx_;
        int num_vertices_;
        float avg_human_speed_;
        float avg_robot_speed_;
    };

  } /* mrn */
  
} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_RESTRICTED_MODEL_H */
