#ifndef BWI_GUIDANCE_SOLVER_UTILS_H
#define BWI_GUIDANCE_SOLVER_UTILS_H

#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/poisson_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>

#include <bwi_tools/common/RNG.h>

namespace bwi_guidance_solver {

  typedef boost::variate_generator<boost::mt19937&, boost::uniform_real<float>
    > URGen;
  typedef boost::shared_ptr<URGen> URGenPtr;

  typedef boost::variate_generator<boost::mt19937&, boost::uniform_int<int> >
    UIGen;
  typedef boost::shared_ptr<UIGen> UIGenPtr;

  typedef boost::variate_generator<boost::mt19937&, 
                                   boost::poisson_distribution<int> > PIGen;
  typedef boost::shared_ptr<PIGen> PIGenPtr;

  inline int select(const std::vector<float>& probabilities, URGenPtr rng) {
    float random_value = (*rng)();
    float prob_sum = probabilities[0];
    for (int i = 1; i < probabilities.size(); ++i) {
      if (random_value < prob_sum) return i - 1;
      prob_sum += probabilities[i];
    }
    return probabilities.size() - 1;
  }
  
  inline int select(const std::vector<float>& probabilities, boost::shared_ptr<RNG> rng) {
    float random_value = rng->randomFloat();
    float prob_sum = probabilities[0];
    for (int i = 1; i < probabilities.size(); ++i) {
      if (random_value < prob_sum) return i - 1;
      prob_sum += probabilities[i];
    }
    return probabilities.size() - 1;
  }
} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_UTILS_H */
