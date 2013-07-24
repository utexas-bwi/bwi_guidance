#ifndef VALUEITERATION_CJEV4VVJ
#define VALUEITERATION_CJEV4VVJ

/*
File: ValueIteration.h
Author: Piyush Khandelwal
Description: Value iteration operating on predictive models 
Created:  2013-07-23
*/

#include <boost/shared_ptr.hpp>
#include <limits>

#include "PredictiveModel.h"
#include "VIEstimator.h"

#ifdef VI_DEBUG
#define VI_OUTPUT(x) std::cout << x << std::endl
#else
#define VI_OUTPUT(x) ((void) 0)
#endif

template<class State, class Action>
class ValueIteration {
public:
  ValueIteration (boost::shared_ptr<PredictiveModel<State, Action> > model,
      boost::shared_ptr<VIEstimator<State, Action> > value_estimator,
      float gamma, unsigned int max_iter);
  virtual ~ValueIteration () {}

  void computePolicy();
  Action getBestAction(const State& state) const;

  virtual std::string generateDescription(unsigned int indentation = 0) {
    return std::string("stub");
  }

private:

  boost::shared_ptr<PredictiveModel<State, Action> > model_;
  boost::shared_ptr<VIEstimator<State, Action> > value_estimator_;

  float gamma_;
  float max_iter_;

  bool policy_available_;

};

template<class State, class Action>
ValueIteration<State, Action>::ValueIteration(
    boost::shared_ptr<PredictiveModel<State, Action> > model,
    boost::shared_ptr<VIEstimator<State, Action> > value_estimator,
    float gamma, unsigned int max_iter) : model_(model), 
  value_estimator_(value_estimator), gamma_(gamma), 
  max_iter_(max_iter), policy_available_(false) {}

template<class State, class Action>
void ValueIteration<State,Action>::computePolicy() {

  bool change = true;
  size_t count = 0;
  while(change && count < max_iter_) {
    change = false;
    count++;
    VI_OUTPUT("Iteration #" << count);
    std::vector<State> states;
    model_->getStateVector(states);
    for (typename std::vector<State>::const_iterator s = states.begin();
        s != states.end(); ++s) {
      const State& state = *s;
      if (model_->isTerminalState(state)) {
        value_estimator_->updateValue(state, 0);
        continue; // nothing to do here, carry on
      }
      std::vector<Action> actions;
      model_->getActionsAtState(state, actions);
      float value = -std::numeric_limits<float>::max();
      Action best_action;
      for (typename std::vector<Action>::const_iterator a = actions.begin();
          a != actions.end(); ++a) {
        const Action& action = *a;
        float action_value = 0;
        std::vector<State> next_states;
        std::vector<float> rewards, probabilities;
        model_->getTransitionDynamics(state, action, next_states, rewards, 
            probabilities);
        for (size_t ns_counter = 0; ns_counter < next_states.size(); 
            ++ns_counter) {
          State& ns = next_states[ns_counter];
          float& reward = rewards[ns_counter];
          float& probability =  probabilities[ns_counter];
          float ns_value = value_estimator_->getValue(ns);
          action_value += probability * (reward + gamma_ * ns_value);
        }

        if (action_value > value) {
          value = action_value;
          best_action = action;
        }
      }
      change = change || (value_estimator_->getValue(state) != value);
      value_estimator_->updateValue(state, value);
      value_estimator_->setBestAction(state, best_action);
    }
  }
}

template<class State, class Action>
Action ValueIteration<State,Action>::getBestAction(const State& state) const {
  return value_estimator_->getBestAction(state);
}

#endif /* end of include guard: VALUEITERATION_CJEV4VVJ */
