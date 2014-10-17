#ifndef BWI_GUIDANCE_SOLVER_IRM_HEURISTIC_SOLVER_H
#define BWI_GUIDANCE_SOLVER_IRM_HEURISTIC_SOLVER_H

#include <bwi_guidance_solver/irm/solver.h>
#include <bwi_guidance_solver/irm/structures.h>

namespace bwi_guidance_solver {

  namespace irm {

    class HeuristicSolver : public Solver {

      public:

        virtual bool initializeSolverSpecific(Json::Value &params);
        virtual Action getBestAction(const State& state);
        virtual std::string getSolverName();

        Action getBestActionWithBlacklistedVertices(const State& state,
            boost::shared_ptr<std::vector<int> > blacklisted_vertices =
            boost::shared_ptr<std::vector<int> >()) const;

        std::map<int, std::vector<int> > visible_vertices_map_;
    };

  } /* irm - InstantaneousRobotMotion */

} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_IRM_HEURISTIC_SOLVER_H */
