#ifndef BWI_GUIDANCE_SOLVER_MRN_SINGLE_ROBOT_SOLVER_H
#define BWI_GUIDANCE_SOLVER_MRN_SINGLE_ROBOT_SOLVER_H

#include <bwi_guidance_solver/mrn/solver.h>
#include <bwi_guidance_solver/mrn/structures.h>

namespace bwi_guidance_solver {

  namespace mrn {

    class SingleRobotSolver : public Solver {

      public:

#define PARAMS(_) \
          _(bool,improved,improved,true)

          Params_STRUCT(PARAMS)
#undef PARAMS


        virtual Action getBestAction(const State& state);
        virtual std::string getSolverName();
        virtual void performEpisodeStartComputation(const State &state);
        virtual void performPostActionComputation(const State &state, float time = 0.0);

      private:

        Params params_;

    };

  } /* mrn */

} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_SINGLE_ROBOT_SOLVER_H */
