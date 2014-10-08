#ifndef BWI_GUIDANCE_SOLVER_IRM_HEURISTIC_SOLVER_H
#define BWI_GUIDANCE_SOLVER_IRM_HEURISTIC_SOLVER_H

#include <bwi_guidance_solver/irm/solver.h>
#include <bwi_guidance_solver/irm/structures.h>

namespace bwi_guidance_solver {

  namespace IRM {

    class HeuristicSolver : Solver {

      public:

        ~HeuristicSolver();

        virtual void initializeSolverSpecific(Json::Value &params);
        virtual void reset(int seed, int goal_idx);
        virtual Action getBestAction(const State& state);

        Action getBestAction(const State& state,
            boost::shared_ptr<std::vector<int> > blacklisted_vertices =
            boost::shared_ptr<std::vector<int> >()) const;

      protected:

        int goal_idx_;

        std::map<int, std::vector<int> > visible_vertices_map_;
    };

  } /* IRM - InstantaneousRobotMotion */

} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_IRM_HEURISTIC_SOLVER_H */
