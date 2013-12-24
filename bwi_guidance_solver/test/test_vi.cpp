#include<fstream>

#include <bwi_guidance_solver/ValueIteration.h>
#include <bwi_guidance_solver/PredictiveModel.h>
#include <boost/foreach.hpp>

class TestModel : public PredictiveModel<int, int>{
public:
  TestModel () {}
  virtual ~TestModel () {}

  virtual bool isTerminalState(const int &state) const {
    return state == 3;
  }
  virtual void getActionsAtState(const int &state, std::vector<int>& actions) {
    actions.clear();
    actions.push_back(0);
  }
  virtual void getStateVector(std::vector<int>& states) {
    states.clear();
    states.push_back(1);
    states.push_back(2);
    states.push_back(3);
  }
  virtual void getTransitionDynamics(const int &state, 
      const int &action, std::vector<int> &next_states, 
      std::vector<float> &rewards, std::vector<float> &probabilities) {
    next_states.clear();
    rewards.clear();
    probabilities.clear();
    if (state == 1) {
      next_states.push_back(2); 
      rewards.push_back(-1);
      probabilities.push_back(0.25);
      next_states.push_back(3); 
      rewards.push_back(-1);
      probabilities.push_back(0.75);
    } else if (state == 2) {
      next_states.push_back(1);
      rewards.push_back(-1);
      probabilities.push_back(1.0);
    }
  }

  virtual std::string generateDescription(unsigned int indentation = 0) {
    return std::string("stub");
  }

};

class TestEstimator : public VIEstimator<int, int> {
  public:
    TestEstimator () {}
    virtual ~TestEstimator () {}

    virtual float getValue(const int &state) {
      return values_[state];
    }
    virtual void updateValue(const int &state, float value) {
      values_[state] = value;
    }
    virtual int getBestAction(const int &state) {
      return actions_[state];
    }
    virtual void setBestAction(const int &state, const int& action) {
      actions_[state] = action;
    }

    virtual void saveEstimatedValues(const std::string& file) {
      std::ofstream fout(file.c_str());
      fout << values_[0] << std::endl;
      fout << values_[1] << std::endl;
      fout << values_[2] << std::endl;
      fout << actions_[0] << std::endl;
      fout << actions_[1] << std::endl;
      fout << actions_[2] << std::endl;
      fout.close();
    }
    virtual void loadEstimatedValues(const std::string& file) {
      std::ifstream fin(file.c_str());
      fin >> values_[0];
      fin >> values_[1];
      fin >> values_[2];
      fin >> actions_[0];
      fin >> actions_[1];
      fin >> actions_[2];
      fin.close();
    }

    virtual std::string generateDescription(unsigned int indentation = 0) {
      return std::string("stub");
    }
  float values_[3];
  float actions_[3];
};

void testValueIteration(std::string file = "") {

  boost::shared_ptr<TestModel> model(new TestModel);
  boost::shared_ptr<TestEstimator> estimator(new TestEstimator);
  ValueIteration<int, int> vi(model, estimator, 1.0, 1e-5, 1000);

  bool policyAvailable = false;
  if (!file.empty()) {
    std::ifstream my_file(file.c_str());
    if (my_file.good()) {
      policyAvailable = true;
    }
  }
  if (policyAvailable) {
    vi.loadPolicy(file);
    std::cout << "Read policy from file: " << file << std::endl;
  } else {
    vi.computePolicy();
    vi.savePolicy("policy.txt");
    std::cout << "Saved policy to file: policy.txt" << std::endl;
  }
}

int main(int argc, char** argv) {

  if (argc >= 2) {
    testValueIteration(std::string(argv[1]));
  } else {
    testValueIteration();
  }

  return 0;
}
