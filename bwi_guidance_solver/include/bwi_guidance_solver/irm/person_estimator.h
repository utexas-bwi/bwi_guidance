#ifndef BWI_GUIDANCE_SOLVER_IRM_PERSON_ESTIMATOR_H
#define BWI_GUIDANCE_SOLVER_IRM_PERSON_ESTIMATOR_H

#include <stdexcept>
#include <map>

#include <bwi_rl/planning/VIEstimator.h>
#include <bwi_guidance_solver/irm/structures.h>

namespace boost {
  namespace serialization {
    class access;
  }
}

namespace bwi_guidance_solver {

  namespace irm {

    class PersonEstimator : public VIEstimator<State, Action> {
      public:

        PersonEstimator () {}
        virtual ~PersonEstimator () {}

        virtual float getValue(const State &state);
        virtual void updateValue(const State &state, float value);
        virtual Action getBestAction(const State &state);
        virtual void setBestAction(const State &state, const Action& action);

        virtual void saveEstimatedValues(const std::string& file);
        virtual void loadEstimatedValues(const std::string& file);

        virtual std::string generateDescription(unsigned int indentation = 0) {
          return std::string("stub");
        }

      private:

        std::map<State, float> value_cache_;
        std::map<State, Action> best_action_cache_;

        friend class boost::serialization::access;
        template<class Archive>
          void serialize(Archive & ar, const unsigned int version);

        float default_value_;
    };

  } /* irm - InstantaneousRobotMotion */

} /* bwi_guidance_solver */


#endif /* end of include guard: BWI_GUIDANCE_SOLVER_IRM_PERSON_ESTIMATOR_H */
