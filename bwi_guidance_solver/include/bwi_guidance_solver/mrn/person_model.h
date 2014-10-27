#ifndef BWI_GUIDANCE_SOLVER_PERSON_MODEL_H
#define BWI_GUIDANCE_SOLVER_PERSON_MODEL_H

#include <nav_msgs/OccupancyGrid.h>
#include <bwi_rl/planning/Model.h>
#include <stdint.h>

#include <bwi_guidance_solver/mrn/structures.h>
#include <bwi_guidance_solver/mrn/transition_model.h>
#include <bwi_mapper/graph.h>
#include <bwi_tools/common/Params.h>

namespace bwi_guidance_solver {

  namespace mrn {
    
    class PersonModel : public Model<State, Action> {

      public:

#define PARAMS(_) \
          _(float,frame_rate,frame_rate,0.0f) \
          _(int,num_robots,num_robots,10) \
          _(float,avg_human_speed,avg_human_speed,1.0f) \
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
        virtual void takeAction(const State &state, 
                                const Action &action, 
                                float &reward, 
                                State &next_state, 
                                bool &terminal, 
                                int &depth_count, 
                                boost::shared_ptr<RNG> rng);

        virtual void getFirstAction(const State &state, Action &action);
        virtual bool getNextAction(const State &state, Action &action);
        virtual void getAllActions(const State &state, std::vector<Action>& actions);

        virtual std::string generateDescription(unsigned int indentation = 0) {
          return std::string("stub");
        }

        /* Same as the inherited takeAction function, but provides more information. */
        void takeAction(const State &state, 
                        const Action &action, 
                        float &reward, 
                        State &next_state, 
                        bool &terminal, 
                        int &depth_count,
                        boost::shared_ptr<RNG> &rng,
                        float &time_loss,
                        float &utility_loss,
                        std::vector<State> &frame_vector);

        /* A convenience function to visualize a model. */
        void drawState(const State& state, cv::Mat& image);
     
      private:

        /* Some private helper functions. */
        void getColocatedRobotIds(const State& state, std::vector<int> &robot_ids);
        void getActionsAtState(const State &state, std::vector<Action>& actions);

        /* Some cached data. */
        std::vector<std::vector<std::vector<size_t> > > shortest_paths_;
        std::vector<std::vector<float> > shortest_distances_;
        std::map<int, std::vector<int> > adjacent_vertices_map_;

        /* The three subcomponents of the transition model. */
        const TaskGenerationModel::Ptr task_generation_model_;
        const MotionModel::Ptr motion_model_;
        const HumanDecisionModel::Ptr human_decision_model_;

        bwi_mapper::Graph graph_;
        nav_msgs::OccupancyGrid map_;

        Params params_;
        int goal_idx_;
        int num_vertices_;
    };

  } /* mrn */
  
} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_PERSON_MODEL_H */
