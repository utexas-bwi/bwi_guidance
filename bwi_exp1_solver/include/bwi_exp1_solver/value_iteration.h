/** \file ValueIteration2.hh
    Defines the ValueIteration2 class
    \author Todd Hester
*/

#ifndef VALUE_ITERATION_XOO8QINS
#define VALUE_ITERATION_XOO8QINS

#include <rl_common/Random.h>
#include <rl_common/core.hh>

#include <set>
#include <vector>
#include <map>
#include <stdint.h>

/** Planner that performs value iteration to compute a policy based on a model */
class ValueIteration2 : public Planner {
public:

  /** Standard constructor
      \param numactions numactions in the domain
      \param gamma discount factor
      \param MAX_LOOPS maximum number of iterations of VI we'll run
      \param MAX_TIME maximum time we'll allow VI to run
      \param modelType specifies model type
      \param featmax maximum value of each feature
      \param featmin minimum value of each feature
      \param statesPerDim # of values to discretize each feature into
      \param rng random number generator
  */
  ValueIteration2(int numactions, float gamma,
                 int MAX_LOOPS, float MAX_TIME, int modelType,
                 const std::vector<float> &featmax, 
                 const std::vector<float> &featmin, 
                 const std::vector<int> &statesPerDim);

  /** Unimplemented copy constructor: internal state cannot be simply
      copied. */
  ValueIteration2(const ValueIteration2 &);

  virtual ~ValueIteration2();

  virtual void setModel(MDPModel* model);
  virtual bool updateModelWithExperience(const std::vector<float> &last, 
                                         int act, 
                                         const std::vector<float> &curr, 
                                         float reward, bool term);
  virtual void planOnNewModel();
  virtual int getBestAction(const std::vector<float> &s);
  virtual void savePolicy(const char* filename);

  /** Initialize the states for this domain (based on featmin and featmax) */
  void initStates();

  /** Fill in a state based on featmin and featmax */
  void fillInState(std::vector<float>s, int depth);

  // bool PLANNERDEBUG;
  // bool POLICYDEBUG;
  // bool MODELDEBUG;
  // bool ACTDEBUG;

  /** MDPModel that we're using with planning */
  MDPModel* model;

  /** The implementation maps all sensations to a set of canonical
      pointers, which serve as the internal representation of
      environment state. */
  typedef const std::vector<float> *state_t;


protected:


  struct state_info;
  struct model_info;

  /** State info struct. Maintains visit counts, models, and q-values for state-actions. */
  struct state_info {

    int id;

    // experience data
    std::vector<int> visits;

    // data filled in from models
    std::vector<StateActionInfo> modelInfo;

    // q values from policy creation
    std::vector<float> Q;

    // number of actions available at this state
    uint32_t numactions;

  };

  /** Produces a canonical representation of the given sensation.
      \param s The current sensation from the environment.
      \return A pointer to an equivalent state in statespace. */
  state_t canonicalize(const std::vector<float> &s);

  /** Initialize a new state */
  void initNewState(state_t s);

  /** Compuate a policy from a model */
  void createPolicy();

  /** Print information for each state. */
  void printStates();

  /** Calculate which states are reachable from states the agent has actually visited. */
  void calculateReachableStates();

  /** Remove states from set that were deemed unreachable. */
  void removeUnreachableStates();

  /** Update the tabular copy of our model from the MDPModel */
  void updateStatesFromModel();

  /** Update a given state-actions model in its state_info struct from the MDPModel */
  void updateStateActionFromModel(const std::vector<float> &state, int j);

  /** Get the current time in seconds */
  double getSeconds();

  /** Return a discretized version of the input state. */
  std::vector<float> discretizeState(const std::vector<float> &s);

private:

  /** Set of all distinct sensations seen.  Pointers to elements of
      this set serve as the internal representation of the environment
      state. */
  std::set<std::vector<float> > statespace;

  /** Hashmap mapping state vectors to their state_info structs. */
  std::map<state_t, state_info> statedata;

  std::vector<float> featmax;
  std::vector<float> featmin;

  std::vector<float> prevstate;
  int prevact;

  double planTime;

  int nstates;
  int nactions; 
  
  int MAX_STEPS;
  bool timingType;

  const int numactions;
  const float gamma;

  const int MAX_LOOPS;
  const float MAX_TIME;
  const int modelType;
  const std::vector<int> statesPerDim;

};

#endif /* end of include guard: VALUE_ITERATION_XOO8QINS */
