#ifndef MODEL_JY7VM3ZW
#define MODEL_JY7VM3ZW

namespace rl_suite {

  class MDPModel {

    public:

      /** Update the MDP model with a vector of experiences. */
      virtual bool updateWithExperiences(std::vector<experience> &instances) = 0;

      /** Update the MDP model with a single experience. */
      virtual bool updateWithExperience(experience &instance) = 0;

      /** Get the predictions of the MDP model for a given state action */
      virtual float getStateActionInfo(const std::vector<float> &state, 
          int action, StateActionInfo* retval) = 0;

      /** Get a copy of the MDP Model */
      virtual PersonModel* getCopy() = 0;
      virtual ~PersonModel() {};

  };

} /* rl_suite */


#endif /* end of include guard: MODEL_JY7VM3ZW */
