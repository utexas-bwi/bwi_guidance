#ifndef BWI_GUIDANCE_SOLVER_MRN_SINGLE_ROBOT_SOLVER_H
#define BWI_GUIDANCE_SOLVER_MRN_SINGLE_ROBOT_SOLVER_H

#include <bwi_guidance_solver/mrn/solver.h>
#include <bwi_guidance_solver/mrn/extended_structures.h>
#include <bwi_rl/planning/DefaultPolicy.h>

namespace bwi_guidance_solver {

  namespace mrn {

    class SingleRobotSolver : public Solver, public DefaultPolicy<ExtendedState, Action> {

      public:

        /* Inherited from bwi_guidance_solver::mrn::Solver */
        virtual Action getBestAction(const ExtendedState& state);

        /* Inherited from DefaultPolicy - This function needs to be const qualified, as it is called
         * from multi-threaded code. At the very least, it should never use rng_, which has threading
         * issues. */
        virtual int getBestAction(const ExtendedState& state, 
                                  const std::vector<Action> &actions, 
                                  const boost::shared_ptr<RNG> &rng);

        virtual std::string getSolverName();
        virtual bool initializeSolverSpecific(Json::Value &params);
        virtual void resetSolverSpecific();
        virtual void performEpisodeStartComputation(const ExtendedState &state);
        virtual void performPostActionComputation(const ExtendedState &state, float time, bool new_action);

      private:

        boost::shared_ptr<RNG> rng_;
        std::vector<std::vector<std::vector<size_t> > > shortest_paths_;
        std::vector<std::vector<float> > shortest_distances_;
        std::map<int, std::vector<int> > adjacent_vertices_map_;

    };

  } /* mrn */

} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_SINGLE_ROBOT_SOLVER_H */
