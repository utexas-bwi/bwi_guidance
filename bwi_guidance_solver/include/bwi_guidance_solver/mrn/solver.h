#ifndef BWI_GUIDANCE_SOLVER_MRN_SOLVER_H
#define BWI_GUIDANCE_SOLVER_MRN_SOLVER_H

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <vector>

#include <bwi_guidance_solver/mrn/domain.h>
#include <bwi_guidance_solver/mrn/person_model.h>
#include <bwi_mapper/graph.h>
#include <bwi_tools/common/Params.h>

namespace bwi_guidance_solver {

  namespace mrn {

    class Solver {

      public:

        bool initialize(Domain::Params &domain_params, 
                        Json::Value &params, 
                        const nav_msgs::OccupancyGrid &map,
                        const bwi_mapper::Graph &graph, 
                        const std::vector<int> &robot_home_base,
                        const std::string &base_directory);

        void reset(int seed, int goal_idx);

        std::map<std::string, std::string> getParamsAsMap();

        virtual Action getBestAction(const State &state) = 0;
        virtual std::string getSolverName() = 0;

        virtual bool initializeSolverSpecific(Json::Value &params);
        virtual void resetSolverSpecific();
        virtual void precomputeAndSavePolicy(int problem_identifier);
        virtual void performEpisodeStartComputation(const State &state);
        virtual void performPostActionComputation(const State &state, float time = 0.0);
        virtual std::map<std::string, std::string> getParamsAsMapSolverSpecific();

      protected:

        /* Some test instance/precomputation specific pieces of information */
        boost::shared_ptr<PersonModel> model_;
        int seed_;
        int goal_idx_;

        /* Some general pieces of information required by the solver */
        std::string base_directory_;
        nav_msgs::OccupancyGrid map_;
        bwi_mapper::Graph graph_;
        std::vector<int> robot_home_base_;

        Domain::Params domain_params_;

    };

  } /* irm - InstantaneousRobotMotion */

} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_SOLVER_H */
