#ifndef BWI_GUIDANCE_SOLVER_MRN_HEURISTIC_SOLVER_H
#define BWI_GUIDANCE_SOLVER_MRN_HEURISTIC_SOLVER_H

#include <bwi_guidance_solver/irm/heuristic_solver.h>
#include <bwi_guidance_solver/mrn/structures.h>
#include <bwi_guidance_solver/mrn/person_model.h>

namespace bwi_guidance_solver {

  namespace mrn {

    class HeuristicSolver : public irm::HeuristicSolver {

      public:
        HeuristicSolver(const nav_msgs::OccupancyGrid& map, 
                        const bwi_mapper::Graph& graph, int goal_idx, bool improved = false,
                        float human_speed = 1.0f);
        ~HeuristicSolver();
        void computePolicy();
        void loadPolicy(const std::string& file);
        void savePolicy(const std::string& file);
        Action getBestAction(const bwi_guidance::State& state, const
                             boost::shared_ptr<bwi_guidance::PersonModel>& evaluation_model);
        virtual std::string generateDescription(unsigned int indentation = 0) {
          return std::string("stub");
        }

      protected:
        bool improved_;
        float human_speed_;
    };

  } /* mrn */

} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_HEURISTIC_SOLVER_H */
