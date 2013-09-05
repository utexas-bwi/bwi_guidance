#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <fstream> 

#include <bwi_exp1_solver/person_estimator2.h>

namespace bwi_exp1 {

  float PersonEstimator2::getValue(const State2 &state) {
    return value_cache_[state];
  }

  void PersonEstimator2::updateValue(const State2 &state, float value) {
    value_cache_[state] = value;
  }

  Action PersonEstimator2::getBestAction(const State2 &state) {
    return best_action_cache_[state];
  }

  void PersonEstimator2::setBestAction(const State2 &state, 
      const Action& action) {
    best_action_cache_[state] = action;
  }

  void PersonEstimator2::loadEstimatedValues(const std::string& file) {
    std::ifstream ifs(file.c_str());
    boost::archive::text_iarchive ia(ifs);
    ia >> *this;
  }

  void PersonEstimator2::saveEstimatedValues(const std::string& file) {
    std::ofstream ofs(file.c_str());
    boost::archive::text_oarchive oa(ofs);
    oa << *this;
  }

  template<class Archive>
  void PersonEstimator2::serialize(Archive & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(value_cache_);
    ar & BOOST_SERIALIZATION_NVP(best_action_cache_);
  }

} /* bwi_exp1 */
