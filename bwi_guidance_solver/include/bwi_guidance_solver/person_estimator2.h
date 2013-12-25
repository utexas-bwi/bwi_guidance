#ifndef PERSON_ESTIMATOR2_O2FD6383
#define PERSON_ESTIMATOR2_O2FD6383

#include <boost/lexical_cast.hpp>
#include <stdexcept>
#include <map>

#include <rl_pursuit/planning/VIEstimator.h>
#include <bwi_guidance_solver/structures.h>

namespace boost {
  namespace serialization {
    class access;
  }
}

namespace bwi_guidance {

  class PersonEstimator2 : public VIEstimator<State, Action> {
    public:

      PersonEstimator2 () {}
      virtual ~PersonEstimator2 () {}

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

} /* bwi_guidance */

#endif /* end of include guard: PERSON_ESTIMATOR2_O2FD6383 */
