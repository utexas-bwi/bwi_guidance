#ifndef BWI_GUIDANCE_SOLVER_IRM_VI_SOLVER_H
#define BWI_GUIDANCE_SOLVER_IRM_VI_SOLVER_H

#include <bwi_guidance_solver/irm/person_estimator.h>
#include <bwi_guidance_solver/irm/solver.h>
#include <bwi_rl/planning/ValueIteration.h>

namespace bwi_guidance_solver {

  namespace irm {

    class VISolver : public Solver {

      public:

        virtual bool initializeSolverSpecific(Json::Value &params);
        virtual void reset();
        virtual Action getBestAction(const State &state);

      protected:
        
        boost::shared_ptr<PersonEstimator> estimator_;
        boost::shared_ptr<ValueIteration<State, Action> > vi_;

        ValueIteration<State, Action>::Params vi_params_;

    };

  }

}

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_IRM_VI_SOLVER_H */
