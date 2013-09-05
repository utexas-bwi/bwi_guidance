#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <fstream> 

#include <bwi_exp1_solver/person_estimator.h>

namespace bwi_exp1 {

  float PersonEstimator::getValue(const state_t &state) {
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

  action_t PersonEstimator::getBestAction(const state_t &state) {
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

  void PersonEstimator::loadEstimatedValues(const std::string& file) {
    std::ifstream ifs(file.c_str());
    boost::archive::text_iarchive ia(ifs);
    ia >> *this;
  }

  void PersonEstimator::saveEstimatedValues(const std::string& file) {
    std::ofstream ofs(file.c_str());
    boost::archive::text_oarchive oa(ofs);
    oa << *this;
  }

  template<class Archive>
  void PersonEstimator::serialize(Archive & ar, const unsigned int version) {
    ar & value_cache_;
    ar & best_action_cache_;
  }

} /* bwi_exp1 */
