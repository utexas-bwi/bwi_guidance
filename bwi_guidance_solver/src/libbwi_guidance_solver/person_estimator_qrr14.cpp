#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <fstream> 

#include <bwi_guidance_solver/person_estimator_qrr14.h>

namespace bwi_guidance {

  float PersonEstimatorQRR14::getValue(const StateQRR14 &state) {
    return value_cache_[state];
  }

  void PersonEstimatorQRR14::updateValue(const StateQRR14 &state, float value) {
    value_cache_[state] = value;
  }

  ActionQRR14 PersonEstimatorQRR14::getBestAction(const StateQRR14 &state) {
    return best_action_cache_[state];
  }

  void PersonEstimatorQRR14::setBestAction(const StateQRR14 &state, 
      const ActionQRR14& action) {
    best_action_cache_[state] = action;
  }

  void PersonEstimatorQRR14::loadEstimatedValues(const std::string& file) {
    std::ifstream ifs(file.c_str());
    boost::archive::binary_iarchive ia(ifs);
    ia >> *this;
  }

  void PersonEstimatorQRR14::saveEstimatedValues(const std::string& file) {
    std::ofstream ofs(file.c_str());
    boost::archive::binary_oarchive oa(ofs);
    oa << *this;
  }

  template<class Archive>
  void PersonEstimatorQRR14::serialize(Archive & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(value_cache_);
    ar & BOOST_SERIALIZATION_NVP(best_action_cache_);
  }

} /* bwi_guidance */
