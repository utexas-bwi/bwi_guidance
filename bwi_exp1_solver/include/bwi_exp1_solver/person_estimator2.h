#ifndef PERSON_ESTIMATOR2_O2FD6383
#define PERSON_ESTIMATOR2_O2FD6383

#include <boost/lexical_cast.hpp>
#include <stdexcept>
#include <map>

#include <bwi_exp1_solver/VIEstimator.h>
#include <bwi_exp1_solver/structures.h>

namespace boost {
  namespace serialization {
    class access;
  }
}

namespace bwi_exp1 {

  class PersonEstimator2 : public VIEstimator<State2, Action> {
    public:

      PersonEstimator2 () {}
      virtual ~PersonEstimator2 () {}

      virtual float getValue(const State2 &state);
      virtual void updateValue(const State2 &state, float value);
      virtual Action getBestAction(const State2 &state);
      virtual void setBestAction(const State2 &state, const Action& action);

      virtual void saveEstimatedValues(const std::string& file);
      virtual void loadEstimatedValues(const std::string& file);

      virtual std::string generateDescription(unsigned int indentation = 0) {
        return std::string("stub");
      }

    private:

      std::map<State2, float> value_cache_;
      std::map<State2, Action> best_action_cache_;

      friend class boost::serialization::access;
      template<class Archive>
      void serialize(Archive & ar, const unsigned int version);

      float default_value_;
  };

} /* bwi_exp1 */

#endif /* end of include guard: PERSON_ESTIMATOR2_O2FD6383 */