#ifndef BWI_GUIDANCE_SOLVER_MRN_DOMAIN_H
#define BWI_GUIDANCE_SOLVER_MRN_DOMAIN_H

#include <boost/shared_ptr.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <vector>

#include <pluginlib/class_loader.h>

#include <bwi_mapper/graph.h>
#include <bwi_rl/planning/domain.h>
#include <bwi_tools/common/Params.h>

namespace bwi_guidance_solver {

  namespace mrn {

    class Solver;

    class Domain : public bwi_rl::Domain {

      public:

#define PARAMS(_) \
          _(float,frame_rate,frame_rate,0.0f) \
          _(bool,save_video,save_video,false) \
          _(bool,start_colocated,start_colocated,true) \
          _(int,max_robots,max_robots,10) \
          _(float,distance_limit,distance_limit,300.0f) \
          _(float,human_speed,human_speed,1.0f) \
          _(float,robot_speed,robot_speed,0.5f) \
          _(float,utility_multiplier,utility_multiplier,1.0f) \
          _(bool,use_shaping_reward,use_shaping_reward,true) \
          _(bool,discourage_bad_assignments,discourage_bad_assignments,false) \
          _(std::string,map_file,map_file,"") \
          _(std::string,graph_file,graph_file,"")

          Params_STRUCT(PARAMS)
#undef PARAMS

        ~Domain();

        virtual bool initialize(Json::Value &experiment, const std::string &base_directory);
        virtual void precomputeAndSavePolicy(int problem_identifier);
        virtual void testInstance(int seed);

      private:

        Params params_;
        std::string base_directory_;

        std::vector<boost::shared_ptr<Solver> > solvers_;
        boost::shared_ptr<pluginlib::ClassLoader<Solver> > class_loader_;

        nav_msgs::OccupancyGrid map_;
        bwi_mapper::Graph graph_;
    };

  } /* mrn - Multi Robot with Navigation */

} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_IRM_DOMAIN_H */
