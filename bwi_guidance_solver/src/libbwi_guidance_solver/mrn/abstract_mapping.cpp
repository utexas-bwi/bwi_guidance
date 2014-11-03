#include <boost/foreach.hpp>
#include <bwi_guidance_solver/mrn/abstract_mapping.h>

namespace bwi_guidance_solver {

  namespace mrn {
    
    void AbstractMapping::map(ExtendedState &state) {
      state.loc_p = 0.0f;
      BOOST_FOREACH(RobotState &r, state.robots) {
        r.loc_u = (r.loc_p > 0.5f) ? r.loc_v : r.loc_u;
        r.loc_p = 0.0f;
        r.loc_v = 0.0f;
      }
    }

  } /* mrn */
  
} /* bwi_guidance_solver */


