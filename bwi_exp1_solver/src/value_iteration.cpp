/** \file ValueIteration2.cc
  Implements the ValueIteration2 class
  \author Todd Hester
  */

#include <bwi_exp1_solver/value_iteration.h>
#include <algorithm>

#include <sys/time.h>


ValueIteration2::ValueIteration2(int numactions, float gamma,
    int MAX_LOOPS, float MAX_TIME, int modelType,
    const std::vector<float> &fmax, 
    const std::vector<float> &fmin, 
    const std::vector<int> &n, Random newRng):
  numactions(numactions), gamma(gamma),
  MAX_LOOPS(MAX_LOOPS), MAX_TIME(MAX_TIME), modelType(modelType),
  statesPerDim(n) {

    rng = newRng;

    nstates = 0;
    nactions = 0;

    timingType = false; //true;

    model = NULL;
    planTime = getSeconds();

    // algorithm options
    MAX_STEPS = 100; 

    featmax = fmax;
    featmin = fmin;

    if (statesPerDim[0] > 0){
      cout << "Planner VI using discretization of " << statesPerDim[0] << endl;
    }
  }

ValueIteration2::~ValueIteration2() { }

void ValueIteration2::setModel(MDPModel* m){
  model = m;
}

// canonicalize all the states so we already have them in our statespace
void ValueIteration2::initStates(){
  cout << "init states" << endl;
  std::vector<float> s(featmin.size());

  fillInState(s,0);
  cout << "init states complete" << endl;
}

void ValueIteration2::fillInState(std::vector<float>s, int depth){

  // if depth == size, canonicalize and return
  if (depth == (int)featmin.size()){
    canonicalize(s);
    return;
  }

  // go through all features at depth
  for (float i = featmin[depth]; i < featmax[depth]+1; i++){
    s[depth] = i;
    fillInState(s, depth+1);
  }
}


/////////////////////////////
// Functional functions :) //
/////////////////////////////


void ValueIteration2::initNewState(state_t s) {
#ifdef PLANNERDEBUG
  cout << "initNewState(s = " << s
    << ") size = " << s->size() << endl;
#endif

#ifdef MODELDEBUG
  cout << "New State: " << endl;
#endif

  // create state info and add to hash map
  state_info* info = &(statedata[s]);

  info->id = nstates++;

#ifdef PLANNERDEBUG
  cout << " id = " << info->id << endl;
#endif

  // init these from model
  info->modelInfo.clear();
  for (int i = 0; i < numactions; i++){
    StateActionInfo act_info;
    float exists = model->getStateActionInfo(*s, i, &act_info);
    if (exists < 0) {
      break;
    }
    info->modelInfo.push_back(act_info);
  }

  info->numactions = info->modelInfo.size();

  // model q values, visit counts
  info->visits.resize(info->numactions, 0);
  info->Q.resize(info->numactions, 0);

  for (int i = 0; i < info->numactions; i++){
    info->Q[i] = rng.uniform(0,0.01);
  }

#ifdef PLANNERDEBUG
  cout << "done with initNewState()" << endl;
#endif
}

bool ValueIteration2::updateModelWithExperience(
    const std::vector<float> &laststate,
    int lastact, const std::vector<float> &currstate,
    float reward, bool term) {
#ifdef PLANNERDEBUG
  cout << "updateModelWithExperience(last = " << &laststate
    << ", curr = " << &currstate
    << ", lastact = " << lastact
    << ", r = " << reward
    << ")" << endl;
#endif

  if (!timingType)
    planTime = getSeconds();

  // canonicalize these things
  state_t last = canonicalize(laststate);
  state_t curr = canonicalize(currstate);

  prevstate = laststate;
  prevact = lastact;

  // if not transition to terminal
  if (curr == NULL)
    return false;

  // get state info
  state_info* info = &(statedata[last]);

  // update the state visit count
  info->visits[lastact]++;

  // init model?
  if (model == NULL){
    cout << "ERROR IN MODEL OR MODEL SIZE" << endl;
    exit(-1);
  }

  experience e;
  e.s = laststate;
  e.next = currstate;
  e.act = lastact;
  e.reward = reward;
  e.terminal = term;
  bool modelChanged = model->updateWithExperience(e);

#ifdef PLANNERDEBUG
  cout << "VI Added exp: " << modelChanged << endl;
#endif

  if (timingType)
    planTime = getSeconds();

  return modelChanged;

}

void ValueIteration2::updateStateActionFromModel(const std::vector<float> &state, int a){

#ifdef PLANNERDEBUG
  cout << "updateStateActionFromModel()" << endl;
#endif

  state_t s = canonicalize(state);

  // get state's info
  state_info* info = &(statedata[s]);

  // update state info
  // get state action info for each action
  StateActionInfo act_info;
  float exists = model->getStateActionInfo(*s, a, &act_info);
  if (exists >= 0) { 
    info->modelInfo[a] = act_info;
  }

}

void ValueIteration2::updateStatesFromModel() {

#ifdef PLANNERDEBUG
  cout << "updateStatesFromModel()" << endl;
#endif

  // for each state
  for (std::set<std::vector<float> >::iterator i = statespace.begin();
      i != statespace.end(); i++){

#ifdef PLANNERDEBUG
    cout << "updateStatesFromModel i = " << &(*i) << endl;
    cout << "State is ";
    for (unsigned j = 0; j < (*i).size(); j++){
      cout << (*i)[j] << ", ";
    }
    cout << endl;
#endif

    state_t s = canonicalize(*i);

    // get state's info
    state_info* info = &(statedata[s]);

    // update state info
    // get state action info for each action
    for (int j = 0; j < numactions; j++){
      StateActionInfo act_info;
      float exists = model->getStateActionInfo(*s, j, &act_info);
      if (exists >= 0) { 
        info->modelInfo[j] = act_info;
      }
    }

  }

#ifdef PLANNERDEBUG
  cout << "updateStatesFromModel " << " totally complete" << endl;
#endif

}

int ValueIteration2::getBestAction(const std::vector<float> &state){
#ifdef PLANNERDEBUG
  cout << "getBestAction(s = " << &state
    << ")" << endl;
#endif


  state_t s = canonicalize(state);

  // get state info
  state_info* info = &(statedata[s]);

  // Get Q values
  std::vector<float> &Q = info->Q;

  // Choose an action
  const std::vector<float>::iterator a =
    random_max_element(Q.begin(), Q.end()); // Choose maximum

  int act = a - Q.begin();
  float val = *a;

#ifdef ACTDEBUG
  cout << endl << "chooseAction State " << (*s)[0] << "," << (*s)[1]
    << " act: " << act << " val: " << val << endl;
  for (int iAct = 0; iAct < numactions; iAct++){
    cout << " Action: " << iAct
      << " val: " << Q[iAct]
      << " visits: " << info->visits[iAct]
      << " modelsAgree: " << info->modelInfo[iAct].known << endl;
  }
#endif

  nactions++;

  // return index of action
  return act;
}




// use VI to compute new policy using model
void ValueIteration2::createPolicy(){

#if defined(PLANNERDEBUG) || defined(POLICYDEBUG)
  cout << endl << "createPolicy()" << endl;
#endif

  float maxError = 5000;
  int nloops = 0;

  float MIN_ERROR = 0.0001;
  //float initTime = getSeconds();
  //cout << "max time " << MAX_TIME  << " max loops: " << MAX_LOOPS << endl;
  int statesUpdated = 0;

  // until convergence (always at least MIN_LOOPS)
  while (maxError > MIN_ERROR){ // && nloops < MAX_LOOPS){

#ifdef POLICYDEBUG
    cout << "max error: " << maxError << " nloops: " << nloops
      << endl;
#endif

    maxError = 0;
    nloops++;

    // for all states
    for (std::set<std::vector<float> >::iterator i = statespace.begin();
        i != statespace.end(); i++){

      statesUpdated++;
      state_t s = canonicalize(*i);

      // get state's info
      state_info* info = &(statedata[s]);

#ifdef POLICYDEBUG
      cout << endl << " State: id: " << info->id << ": " ;
      for (unsigned si = 0; si < s->size(); si++){
        cout << (*s)[si] << ",";
      }
#endif

      // for each action
      for (int act = 0; act < info->numactions; act++){

        // get state action info for this action
        StateActionInfo *modelInfo = &(info->modelInfo[act]);

#ifdef POLICYDEBUG
        cout << "  Action: " << act
          << " State visits: " << info->visits[act]
          << " reward: " << modelInfo->reward 
          << " term: " << modelInfo->termProb << endl;
#endif

        // Q = R + discounted val of next state
        // this is the R part :)
        float newQ = modelInfo->reward;

        float probSum = modelInfo->termProb;

        // for all next states, add discounted value appropriately
        // loop through next state's that are in this state-actions list
        for (std::map<std::vector<float>, float>::iterator outIt
            = modelInfo->transitionProbs.begin();
            outIt != modelInfo->transitionProbs.end(); outIt++){

          std::vector<float> nextstate = (*outIt).first;

#ifdef POLICYDEBUG
          cout << "  Next state was: ";
          for (unsigned oi = 0; oi < nextstate.size(); oi++){
            cout << nextstate[oi] << ",";
          }
          cout << endl;
#endif

          // get transition probability
          float transitionProb = (1.0-modelInfo->termProb) *
            modelInfo->transitionProbs[nextstate];

          probSum += transitionProb;

#ifdef POLICYDEBUG
          cout << "   prob: " << transitionProb << endl;
#endif

          if (transitionProb < 0 || transitionProb > 1.0001){
            cout << "Error with transitionProb: " << transitionProb << endl;
            exit(-1);
          }

          // if there is some probability of this transition
          if (transitionProb > 0.0){

            float maxval = 0.0;

            // make sure its a real state
            bool realState = true;

            for (unsigned b = 0; b < nextstate.size(); b++){
              if (nextstate[b] < (featmin[b]-EPSILON)
                  || nextstate[b] > (featmax[b]+EPSILON)){
                realState = false;
#ifdef POLICYDEBUG
                cout << "    Next state is not valid (feature "
                  << b << " out of range)" << endl;
#endif
                break;
              }
            }

            state_t next;

            // update q values for any states within MAX_STEPS of visited states
            if (!realState){
              next = s;
            } else {
              next = canonicalize(nextstate);
            }

            state_info* nextinfo = &(statedata[next]);

            // find the max value of this next state
            std::vector<float>::iterator maxAct =
              std::max_element(nextinfo->Q.begin(),
                  nextinfo->Q.end());
            maxval = *maxAct;

            nextstate.clear();

#ifdef POLICYDEBUG
            cout << "    Max value: " << maxval << endl;
#endif

            // update q value with this value
            newQ += (gamma * transitionProb * maxval);

          } // transition probability > 0

        } // outcome loop


        if (probSum < 0.9999 || probSum > 1.0001){
          cout << "Error: transition probabilities do not add to 1: Sum: "
            << probSum << endl;
          exit(-1);
        }


        // set q value
        float tdError = fabs(info->Q[act] - newQ);
#ifdef POLICYDEBUG
        cout << "  NewQ: " << newQ
          << " OldQ: " << info->Q[act] << endl;
#endif
        info->Q[act] = newQ;

        // check max error
        if (tdError > maxError)
          maxError = tdError;

#ifdef POLICYDEBUG
        cout << "  TD error: " << tdError
          << " Max error: " << maxError << endl;
#endif

      } // action loop

    } // state loop

  } // while not converged loop

  if (nloops >= MAX_LOOPS){
    cout << nactions << " Policy creation ended with maxError: " << maxError
      << " nloops: " << nloops << " time: " << (getSeconds()-planTime)
      << " states: " << statesUpdated
      << endl;
  }

#ifdef POLICYDEBUG
  cout << nactions
    << " policy creation complete: maxError: "
    << maxError << " nloops: " << nloops
    << endl;
#endif

  }

  void ValueIteration2::planOnNewModel(){

    // update model info
    // can just update one for tabular model
    if (modelType == RMAX){
      updateStateActionFromModel(prevstate, prevact);
    }
    else {
      updateStatesFromModel();
    }

    // run value iteration
    createPolicy();

  }

  ////////////////////////////
  // Helper Functions       //
  ////////////////////////////

  ValueIteration2::state_t ValueIteration2::canonicalize(const std::vector<float> &s) {
    cout << "canonicalize(s = " << s[0] << ", "
      << s[1] << ")" << endl;

    std::vector<float> s2;
    if (statesPerDim[0] > 0){
      s2 = discretizeState(s);
    } else {
      s2 = s;
    }

#ifdef PLANNERDEBUG
    cout << "discretized(" << s2[0] << ", " << s2[1] << ")" << endl;
#endif

    // get state_t for pointer if its in statespace
    const std::pair<std::set<std::vector<float> >::iterator, bool> result =
      statespace.insert(s2);
    state_t retval = &*result.first; // Dereference iterator then get pointer

#ifdef PLANNERDEBUG
    cout << " returns " << retval
      << " New: " << result.second << endl;
#endif

    // if not, init this new state
    if (result.second) { // s is new, so initialize Q(s,a) for all a
      initNewState(retval);
#ifdef PLANNERDEBUG
      cout << " New state initialized" << endl;
#endif
    }

    return retval;
  }

  void ValueIteration2::printStates(){

    for (std::set< std::vector<float> >::iterator i = statespace.begin();
        i != statespace.end(); i++){

      state_t s = canonicalize(*i);

      state_info* info = &(statedata[s]);

      cout << "State " << info->id << ": ";
      for (unsigned j = 0; j < s->size(); j++){
        cout << (*s)[j] << ", ";
      }
      cout << endl;

      for (int act = 0; act < info->numactions; act++){
        cout << " visits[" << act << "] = " << info->visits[act]
          << " Q: " << info->Q[act]
          << " R: " << info->modelInfo[act].reward << endl;
      }

    }
  }

  double ValueIteration2::getSeconds(){
    struct timezone tz;
    timeval timeT;
    gettimeofday(&timeT, &tz);
    return  timeT.tv_sec + (timeT.tv_usec / 1000000.0);
  }

  void ValueIteration2::savePolicy(const char* filename){

    ofstream policyFile(filename, ios::out | ios::binary | ios::trunc);

    // first part, save the vector size
    int fsize = featmin.size();
    policyFile.write((char*)&fsize, sizeof(int));

    // save numactions
    policyFile.write((char*)&numactions, sizeof(int));

    // go through all states, and save Q values
    for (std::set< std::vector<float> >::iterator i = statespace.begin();
        i != statespace.end(); i++){

      state_t s = canonicalize(*i);
      state_info* info = &(statedata[s]);

      // save state
      policyFile.write((char*)&((*i)[0]), sizeof(float)*fsize);

      // save q-values
      policyFile.write((char*)&(info->Q[0]), sizeof(float)*info->numactions);

    }

    policyFile.close();
  }


  // should do it such that an already discretized state stays the same
  // mainly the numerical value of each bin should be the average of that bin
  std::vector<float> ValueIteration2::discretizeState(const std::vector<float> &s){
    std::vector<float> ds(s.size());

    for (unsigned i = 0; i < s.size(); i++){

      // since i'm sometimes doing this for discrete domains
      // want to center bins on 0, not edge on 0
      //cout << "feat " << i << " range: " << featmax[i] << " " << featmin[i] << " " << (featmax[i]-featmin[i]) << " n: " << (float)statesPerDim;

      float factor = (featmax[i] - featmin[i]) / (float)statesPerDim[i];
      int bin = 0;
      if (s[i] > 0){
        bin = (int)((s[i]+factor/2) / factor);
      } else {
        bin = (int)((s[i]-factor/2) / factor);
      }

      ds[i] = factor*bin;
      //cout << " factor: " << factor << " bin: " << bin;
      //cout << " Original: " << s[i] << " Discrete: " << ds[i] << endl;
    }

    return ds;
  }
