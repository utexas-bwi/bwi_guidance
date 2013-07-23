#ifndef PREDICTIVEMODEL_KNIP16QL
#define PREDICTIVEMODEL_KNIP16QL

/*
File: PredictiveModel.h
Author: Piyush Khandelwal
Description: an abstract model for planning requiring predictive models 
Created:  2013-07-23
*/

#include <string>
#include <vector>

template<class State, class Action>
class PredictiveModel {
public:
  PredictiveModel () {}
  virtual ~PredictiveModel () {}

  virtual bool isTerminalState(State &state) = 0;
  virtual std::vector<Action>& getActionsAtState(State &state) = 0;
  virtual std::vector<State>& getStateVector() = 0; // should only be implemented by tabular representations 
  virtual void getTransitionDynamics(const State &state, 
      const Action &action, std::vector<State> &next_states, 
      std::vector<float> &rewards, std::vector<float> &probabilities) = 0;

  virtual std::string generateDescription(unsigned int indentation = 0) = 0;
};

#endif /* end of include guard: PREDICTIVEMODEL_KNIP16QL */
