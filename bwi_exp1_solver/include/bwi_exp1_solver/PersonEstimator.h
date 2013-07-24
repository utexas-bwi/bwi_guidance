#ifndef PERSONESTIMATOR_8O13NOI3
#define PERSONESTIMATOR_8O13NOI3

#include <bwi_exp1_solver/StateValueEstimator.h>

class PersonEstimator : StateValueEstimator<state_t, action_t> {
public:
  StateValueEstimator (float default_value = 0) {}
  virtual ~StateValueEstimator () {}

  virtual void reset() = 0;
  virtual float getValue(const State &state) const = 0;
  virtual void updateValue(const State &state, float value) = 0;
  virtual Action getBestAction(const State &state) const = 0;
  virtual void setBestAction(const State &state, const Action& action) = 0;

  virtual std::string generateDescription(unsigned int indentation = 0) = 0;

private:
  std::vector<float> value_cache_;
  std::vector<action_t> best_action_cache_;
};

#endif /* end of include guard: PERSONESTIMATOR_8O13NOI3 */
