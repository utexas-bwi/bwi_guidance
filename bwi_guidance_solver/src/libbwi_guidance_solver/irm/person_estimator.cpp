#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <fstream> 

#include <bwi_guidance_solver/irm/person_estimator.h>

namespace bwi_guidance_solver {

  namespace irm {

    float PersonEstimator::getValue(const State &state) {
      return value_cache_[state];
    }

    void PersonEstimator::updateValue(const State &state, float value) {
      value_cache_[state] = value;
    }

    Action PersonEstimator::getBestAction(const State &state) {
      return best_action_cache_[state];
    }

    void PersonEstimator::setBestAction(const State &state, 
                                             const Action& action) {
      best_action_cache_[state] = action;
    }

    void PersonEstimator::loadEstimatedValues(const std::string& file) {
      std::ifstream ifs(file.c_str());
      boost::archive::binary_iarchive ia(ifs);
      ia >> *this;
    }

    void PersonEstimator::saveEstimatedValues(const std::string& file) {
      std::ofstream ofs(file.c_str());
      boost::archive::binary_oarchive oa(ofs);
      oa << *this;
    }

    template<class Archive>
      void PersonEstimator::serialize(Archive & ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(value_cache_);
        ar & BOOST_SERIALIZATION_NVP(best_action_cache_);
      }

  } /* irm - InstantaneousRobotMotion */

} /* bwi_guidance_solver */
