#include <bwi_exp1_solver/person_estimator.h>

namespace bwi_exp1 {

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
