#ifndef BWI_GUIDANCE_SOLVER_PERSON_MODEL_H
#define BWI_GUIDANCE_SOLVER_PERSON_MODEL_H

#include <stdint.h>
#include <nav_msgs/OccupancyGrid.h>

#include <bwi_guidance_solver/mrn/person_model.h>
#include <bwi_guidance_solver/mrn/restricted_structures.h>
#include <bwi_guidance_solver/mrn/transition_model.h>
#include <bwi_mapper/graph.h>
#include <bwi_rl/planning/Model.h>
#include <bwi_tools/common/Params.h>

namespace bwi_guidance_solver {

  namespace mrn {
    
    class RestrictedModel : public Model<RestrictedState, RestrictedAction> {

      public:

#define PARAMS(_) \
          _(float,frame_rate,frame_rate,0.0f) \
          _(int,num_robots,num_robots,10) \
          _(int,max_assigned_robots,kax_assigned_robots,1) \
          _(float,avg_robot_speed,avg_robot_speed,0.5f)

          Params_STRUCT(PARAMS)
#undef PARAMS

        PersonModel(const bwi_mapper::Graph& graph, 
                    const nav_msgs::OccupancyGrid& map, 
                    int goal_idx,
                    const MotionModel::Ptr &motion_model,
                    const HumanDecisionModel::Ptr &human_decision_model,
                    const TaskGenerationModel::Ptr &task_generation_model,
                    const Params &params);

        /* Functions inherited from PredictiveModel */
        virtual ~PersonModel() {};

        /* Functions inherited from Model */
        virtual void takeAction(const RestrictedState &state, 
                                const RestrictedAction &action, 
                                float &reward, 
                                RestrictedState &next_state, 
                                bool &terminal, 
                                int &depth_count, 
                                boost::shared_ptr<RNG> rng);

        virtual void getFirstAction(const RestrictedState &state, RestrictedAction &action);
        virtual bool getNextAction(const RestrictedState &state, RestrictedAction &action);
        virtual void getAllActions(const RestrictedState &state, std::vector<RestrictedAction>& actions);

        virtual std::string generateDescription(unsigned int indentation = 0) {
          return std::string("stub");
        }

      private:

        /* Underlying base model. */
        boost::shared_ptr<PersonModel> base_model_;

        /* Some cached data. */
        std::vector<std::vector<std::vector<size_t> > > shortest_paths_;
        std::vector<std::vector<float> > shortest_distances_;
        std::map<int, std::vector<int> > adjacent_vertices_map_;

        Params params_;
        int goal_idx_;
        int num_vertices_;
    };

  } /* mrn */
  
} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_PERSON_MODEL_H */
