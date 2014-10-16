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
          _(float,planning_time_multiplier,planning_time_multiplier,1.0) \
          _(bool,use_abstract_mapping,use_abstract_mapping,true)

          Params_STRUCT(PARAMS)
#undef PARAMS

        virtual Action getBestAction(const State &state);
        virtual std::string getSolverName();

        virtual bool initializeSolverSpecific(Json::Value &params);
        virtual void resetSolverSpecific();
        virtual void performEpisodeStartComputation(const State &state);
        virtual void performPostActionComputation(const State &state, float time = 0.0f);
        virtual std::map<std::string, std::string> getParamsAsMapSolverSpecific();

      protected:

        virtual void search(const State &state, float time);
        
        boost::shared_ptr<MultiThreadedMCTS<State, StateHash, Action> > mcts_;
        Params mcts_solver_params_;
        MultiThreadedMCTS<State, StateHash, Action>::Params mcts_params_;

    };

  }

}

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_MCTS_SOLVER_H */
