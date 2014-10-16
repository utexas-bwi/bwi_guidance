#include <bwi_guidance_solver/mrn/abstract_mapping.h>

namespace bwi_guidance_solver {

  namespace mrn {
    
    void AbstractMapping::map(State &state) {
      state.precision = 0.0f;
      state.from_graph_node = 0;
      state.robots.clear();
      for (unsigned int i = 0; i < state.in_use_robots.size(); ++i) {
        state.in_use_robots[i].robot_id = 0;
      }
    }

  } /* mrn */
  
} /* bwi_guidance_solver */


