#ifndef ENV_3N579X5N
#define ENV_3N579X5N

#include <rl_common/Random.h>
#include <rl_common/core.hh>
#include <stdint.h>
#include <topological_mapper/graph.h>

namespace bwi_exp1 {

  const int MAX_ACTIONS = 15;

  typedef topological_mapper::Graph Graph;
  
  class NavigationEnvironment : public Environment {

    public:

      NavigationEnvironment(const Graph& graph, size_t goal_state);

      virtual ~NavigationEnvironment();

      virtual const std::vector<float> &sensation() const;
      virtual float apply(int action);

      /** Calculate the new state and reward for the given force */
      float transition(float force);

      virtual bool terminal() const;
      virtual void reset();

      virtual int getNumActions();
      virtual void getMinMaxFeatures(std::vector<float> *minFeat, std::vector<float> *maxFeat);
      virtual void getMinMaxReward(float* minR, float* maxR);

      /** Set the state vector (for debug purposes) */
      void setSensation(std::vector<float> newS);

      virtual std::vector<experience> getSeedings();

      /** Get an experience for the given state-action */
      experience getExp(float s0, float s1, float s2, float s3, int a);

    private:

      Graph graph_;
      size_t goal_state_;

      std::vector<size_t> state_vector_;

  };

} /* bwi_exp1 */

#endif /* end of include guard: ENV_3N579X5N */
