#include <bwi_guidance_solver/mrn/abstract_mapping.h>

namespace bwi_guidance_solver {

  namespace mrn {
    
    void AbstractMapping::map(State &state) {
      state.precision = 0.0f;
      state.from_graph_node = 0;
      state.robots.clear();
    }

  } /* mrn */
  
} /* bwi_guidance_solver */


