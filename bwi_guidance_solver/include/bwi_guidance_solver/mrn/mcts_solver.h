#ifndef BWI_GUIDANCE_SOLVER_MRN_MCTS_SOLVER_H
#define BWI_GUIDANCE_SOLVER_MRN_MCTS_SOLVER_H

#include <bwi_guidance_solver/mrn/solver.h>
#include <bwi_rl/planning/MultiThreadedMCTS.h>

namespace bwi_guidance_solver {

  namespace mrn {

    class MCTSSolver : public Solver {

      public:

#define PARAMS(_) \
          _(float,initial_planning_time,initial_planning_time,10.0) \
          _(float,planning_time_multiplier,planning_time_multiplier,1.0)

          Params_STRUCT(PARAMS)
#undef PARAMS

        virtual Action getBestAction(const ExtendedState &state);
        virtual std::string getSolverName();

        virtual bool initializeSolverSpecific(Json::Value &params);
        virtual void resetSolverSpecific();
        virtual void performEpisodeStartComputation(const ExtendedState &state);
        virtual void performPostActionComputation(const ExtendedState &state, float time, bool new_action);
        virtual std::map<std::string, std::string> getParamsAsMapSolverSpecific();

      protected:

        virtual void search(const ExtendedState &state, float time);
        
        boost::shared_ptr<MultiThreadedMCTS<ExtendedState, ExtendedStateHash, Action> > mcts_;
        Params mcts_solver_params_;
        RestrictedModel::Params extended_mdp_params_;
        MultiThreadedMCTS<ExtendedState, ExtendedStateHash, Action>::Params mcts_params_;

    };

  }

}

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_MCTS_SOLVER_H */
