#ifndef BWI_GUIDANCE_SOLVER_IRM_DOMAIN_H
#define BWI_GUIDANCE_SOLVER_IRM_DOMAIN_H

#include <boost/shared_ptr.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <vector>

#include <bwi_mapper/graph.h>
#include <bwi_rl/planning/domain.h>
#include <bwi_tools/common/Params.h>

namespace bwi_guidance_solver {

  namespace irm {

    class Solver;

    class Domain : public bwi_rl::Domain {

      public:

#define PARAMS(_) \
          _(float,distance_limit,distance_limit,300.0f) \
          _(bool,allow_robot_current_idx,allow_robot_current_idx,false) \
          _(float,visibility_range,visibility_range,0.0f) \
          _(bool,allow_goal_visibility,allow_goal_visibility,false) \
          _(std::string,map_file,map_file,"") \
          _(std::string,graph_file,graph_file,"")

          Params_STRUCT(PARAMS)
#undef PARAMS

        virtual bool initialize(Json::Value &experiment, const std::string &base_directory);
        virtual void precomputeAndSavePolicy(int problem_identifier);
        virtual void testInstance(int seed);

      private:

        Params params_;
        std::vector<boost::shared_ptr<Solver> > solvers_;

        nav_msgs::OccupancyGrid map_;
        bwi_mapper::Graph graph_;
    };

  } /* irm - InstantaneousRobotMotion */

} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_IRM_DOMAIN_H */
