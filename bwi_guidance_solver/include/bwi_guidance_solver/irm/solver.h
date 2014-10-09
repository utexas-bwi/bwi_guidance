#ifndef BWI_GUIDANCE_SOLVER_IRM_SOLVER_H
#define BWI_GUIDANCE_SOLVER_IRM_SOLVER_H

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

        inline bool initialize(Domain::Params &domain_params, 
                               Json::Value &params, 
                               const nav_msgs::OccupancyGrid &map,
                               const bwi_mapper::Graph &graph, 
                               const std::string &base_directory) {
          domain_params_ = domain_params;
          general_params_.fromJson(params);
          map_ = map;
          graph_ = graph;

          // Compute the base directory
          std::ostringstream parametrized_dir_ss;
          parametrized_dir_ss << std::fixed << std::setprecision(2);
          parametrized_dir_ss << base_directory << "/ros" << general_params_.reward_on_success << 
            "-rs" << general_params_.reward_structure;
          base_directory_ = parametrized_dir_ss.str();

          return this->initializeSolverSpecific(params);
        }

        inline void reset(const boost::shared_ptr<PersonModel> &model, int seed, int goal_idx) {
          seed_ = seed;
          model_ = model;
          goal_idx_ = goal_idx;
          this->resetSolverSpecific();
        }

        virtual std::map<std::string, std::string> getParamsAsMap() { 
          std::map<std::string, std::string> stringMap = this->getParamsAsMapSolverSpecific();
          std::map<std::string, std::string> general_params_map = general_params_.asMap();
          stringMap.insert(general_params_map.begin(), general_params_map.end());
          return stringMap;
        }

        virtual Action getBestAction(const State &state) = 0;

        virtual bool initializeSolverSpecific(Json::Value &params) {}
        virtual bool resetSolverSpecific() {}
        virtual void precomputeAndSavePolicy(int problem_identifier) {}
        virtual void performEpisodeStartComputation() {}
        virtual void performPostActionComputation(float distance = 0.0) {}
        virtual std::map<std::string, std::string> getParamsAsMapSolverSpecific() { 
          return std::map<std::string, std::string>();
        }

        inline bool shouldAddRewardOnSuccess() { return general_params_.reward_on_success; }
        inline RewardStructure getRewardStructure() { 
          return (RewardStructure)(general_params_.reward_structure);
        }

      protected:

        Solver();

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
