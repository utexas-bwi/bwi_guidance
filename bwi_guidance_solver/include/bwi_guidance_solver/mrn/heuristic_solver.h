#ifndef BWI_GUIDANCE_SOLVER_MRN_HEURISTIC_SOLVER_H
#define BWI_GUIDANCE_SOLVER_MRN_HEURISTIC_SOLVER_H

#include <bwi_guidance_solver/irm/heuristic_solver.h>
#include <bwi_guidance_solver/mrn/solver.h>
#include <bwi_guidance_solver/mrn/structures.h>

namespace bwi_guidance_solver {

  namespace mrn {

    class HeuristicSolver : public Solver {

      public:

#define PARAMS(_) \
          _(bool,improved,improved,true)

          Params_STRUCT(PARAMS)
#undef PARAMS


        virtual bool initializeSolverSpecific(Json::Value &params);
        virtual void resetSolverSpecific();
        virtual Action getBestAction(const State& state);
        virtual std::string getSolverName();
        virtual void performEpisodeStartComputation(const State &state);
        virtual void performPostActionComputation(const State &state, float time = 0.0);

      private:

        irm::HeuristicSolver irm_hs_;
        Params params_;

    };

  } /* mrn */

} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_HEURISTIC_SOLVER_H */
