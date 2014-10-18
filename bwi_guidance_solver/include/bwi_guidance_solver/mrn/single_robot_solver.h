#ifndef BWI_GUIDANCE_SOLVER_MRN_SINGLE_ROBOT_SOLVER_H
#define BWI_GUIDANCE_SOLVER_MRN_SINGLE_ROBOT_SOLVER_H

#include <bwi_guidance_solver/mrn/solver.h>
#include <bwi_guidance_solver/mrn/structures.h>
#include <bwi_rl/planning/DefaultPolicy.h>

namespace bwi_guidance_solver {

  namespace mrn {

    class SingleRobotSolver : public Solver, public DefaultPolicy<State, Action> {

      public:

        /* Inherited from bwi_guidance_solver::mrn::Solver */
        virtual Action getBestAction(const State& state);

        /* Inherited from DefaultPolicy - This function needs to be const qualified, as it is called
         * from multi-threaded code. At the very least, it should never use rng_, which has threading
         * issues. */
        virtual int getBestAction(const State& state, 
                                  const std::vector<Action> &actions, 
                                  const boost::shared_ptr<RNG> &rng);

        virtual std::string getSolverName();
        virtual void resetSolverSpecific();
        virtual void performEpisodeStartComputation(const State &state);
        virtual void performPostActionComputation(const State &state, float time = 0.0);

      private:

        boost::shared_ptr<RNG> rng_;

    };

  } /* mrn */

} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_SINGLE_ROBOT_SOLVER_H */
