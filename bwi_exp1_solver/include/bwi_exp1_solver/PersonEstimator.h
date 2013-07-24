#ifndef PERSONESTIMATOR_8O13NOI3
#define PERSONESTIMATOR_8O13NOI3

#include <boost/lexical_cast.hpp>
#include <stdexcept>

#include <bwi_exp1_solver/StateValueEstimator.h>
#include <bwi_exp1_solver/structures.h>

namespace bwi_exp1 {

  class PersonEstimator : public StateValueEstimator<state_t, action_t> {
    public:

      PersonEstimator (size_t state_space_size, float default_value = 0) :
        value_cache_(state_space_size, default_value),
        best_action_cache_(state_space_size) {}
      virtual ~PersonEstimator () {}

      virtual float getValue(const state_t &state) const;
      virtual void updateValue(const state_t &state, float value);
      virtual action_t getBestAction(const state_t &state) const;
      virtual void setBestAction(const state_t &state, const action_t& action);

      virtual std::string generateDescription(unsigned int indentation = 0) {
        return std::string("stub");
      }

    private:

      std::vector<float> value_cache_;
      std::vector<action_t> best_action_cache_;

      float default_value_;
  };

  float PersonEstimator::getValue(const state_t &state) const {
    if (state > value_cache_.size()) {
      throw std::runtime_error("Estimator getValue() error. Requested idx: " +
          boost::lexical_cast<std::string>(state) + " from total size: " + 
          boost::lexical_cast<std::string>(value_cache_.size())); 
    }
    return value_cache_[state];
  }

  void PersonEstimator::updateValue(const state_t &state, float value) {
    if (state > value_cache_.size()) {
      throw std::runtime_error("Estimator updateValue() error. Requested idx: " +
          boost::lexical_cast<std::string>(state) + " from total size: " + 
          boost::lexical_cast<std::string>(value_cache_.size())); 
    }
    value_cache_[state] = value;
  }

  action_t PersonEstimator::getBestAction(const state_t &state) const {
    if (state > best_action_cache_.size()) {
      throw std::runtime_error("Estimator getAction() error. Requested idx: " +
          boost::lexical_cast<std::string>(state) + " from total size: " + 
          boost::lexical_cast<std::string>(value_cache_.size())); 
    }
    return best_action_cache_[state];
  }

  void PersonEstimator::setBestAction(const state_t &state, 
      const action_t& action) {

    if (state > best_action_cache_.size()) {
      throw std::runtime_error("Estimator setAction() error. Requested idx: " +
          boost::lexical_cast<std::string>(state) + " from total size: " + 
          boost::lexical_cast<std::string>(value_cache_.size())); 
    }
    best_action_cache_[state] = action;
  }
} /* bwi_exp1 */

#endif /* end of include guard: PERSONESTIMATOR_8O13NOI3 */
