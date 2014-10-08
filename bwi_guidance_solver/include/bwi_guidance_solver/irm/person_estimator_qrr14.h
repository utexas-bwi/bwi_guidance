#ifndef BWI_GUIDANCE_SOLVER_PERSON_ESTIMATOR_QRR14
#define BWI_GUIDANCE_SOLVER_PERSON_ESTIMATOR_QRR14

#include <boost/lexical_cast.hpp>
#include <stdexcept>
#include <map>

#include <bwi_rl/planning/VIEstimator.h>
#include <bwi_guidance_solver/structures_qrr14.h>

namespace boost {
  namespace serialization {
    class access;
  }
}

namespace bwi_guidance {

  class PersonEstimatorQRR14 : public VIEstimator<StateQRR14, ActionQRR14> {
    public:

      PersonEstimatorQRR14 () {}
      virtual ~PersonEstimatorQRR14 () {}

      virtual float getValue(const StateQRR14 &state);
      virtual void updateValue(const StateQRR14 &state, float value);
      virtual ActionQRR14 getBestAction(const StateQRR14 &state);
      virtual void setBestAction(const StateQRR14 &state, const ActionQRR14& action);

      virtual void saveEstimatedValues(const std::string& file);
      virtual void loadEstimatedValues(const std::string& file);

      virtual std::string generateDescription(unsigned int indentation = 0) {
        return std::string("stub");
      }

    private:

      std::map<StateQRR14, float> value_cache_;
      std::map<StateQRR14, ActionQRR14> best_action_cache_;

      friend class boost::serialization::access;
      template<class Archive>
      void serialize(Archive & ar, const unsigned int version);

      float default_value_;
  };

} /* bwi_guidance */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_PERSON_ESTIMATOR_QRR14 */
