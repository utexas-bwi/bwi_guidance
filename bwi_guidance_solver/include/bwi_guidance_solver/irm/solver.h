#ifndef BWI_GUIDANCE_SOLVER_IRM_SOLVER_H
#define BWI_GUIDANCE_SOLVER_IRM_SOLVER_H

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <vector>

#include <bwi_guidance_solver/irm/domain.h>
#include <bwi_guidance_solver/irm/person_model.h>
#include <bwi_mapper/graph.h>
#include <bwi_tools/common/Params.h>

namespace bwi_guidance_solver {

  namespace irm {

    class Solver {

      public:

#define PARAMS(_) \
          _(bool,reward_on_success,reward_on_success,false) \
          _(int,reward_structure,reward_structure,STANDARD_REWARD)

          Params_STRUCT(PARAMS)
#undef PARAMS

        bool initialize(Domain::Params &domain_params, 
            Json::Value &params, 
            const nav_msgs::OccupancyGrid &map,
            const bwi_mapper::Graph &graph, 
            const std::string &base_directory);

        void reset(const boost::shared_ptr<PersonModel> &model, int seed, int goal_idx);

        std::map<std::string, std::string> getParamsAsMap();

        virtual Action getBestAction(const State &state) = 0;
        virtual std::string getSolverName() = 0;

        virtual bool initializeSolverSpecific(Json::Value &params);
        virtual void resetSolverSpecific();
        virtual void precomputeAndSavePolicy(int problem_identifier);
        virtual void performEpisodeStartComputation(const State &state);
        virtual void performPostActionComputation(const State &state, float distance = 0.0);
        virtual std::map<std::string, std::string> getParamsAsMapSolverSpecific();

        inline bool shouldAddRewardOnSuccess() { return general_params_.reward_on_success; }
        inline RewardStructure getRewardStructure() { 
          return (RewardStructure)(general_params_.reward_structure);
        }

      protected:

        /* Some test instance/precomputation specific pieces of information */
        int seed_;
        int goal_idx_;
        boost::shared_ptr<PersonModel> model_;

        /* Some general pieces of information required by the solver */
        std::string base_directory_;
        nav_msgs::OccupancyGrid map_;
        bwi_mapper::Graph graph_;

        Domain::Params domain_params_;
        Params general_params_;

    };

  } /* irm - InstantaneousRobotMotion */

} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_IRM_SOLVER_H */
