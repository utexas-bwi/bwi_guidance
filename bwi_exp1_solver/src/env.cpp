#include <bwi_exp1_solver/env.h>

namespace bwi_exp1 {

  NavigationEnvironment::NavigationEnvironment(const Graph& graph, size_t goal_state) :
    graph_(graph), goal_state_(goal_state) {}

  NavigationEnvironment::~NavigationEnvironment() {}

  const std::vector<float> &NavigationEnvironment::sensation() const {
    return std::vector<float>(state_vector_.begin(), state_vector_.end());
  }

  float NavigationEnvironment::apply(action) {
    if (action < 0 || action >= MAX_ACTIONS) {
      throw std::runtime_error("Specified action outside range");
    }

  }

  
  
} /* bwi_exp1 */
