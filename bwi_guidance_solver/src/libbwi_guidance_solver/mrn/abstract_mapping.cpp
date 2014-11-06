#include <boost/foreach.hpp>
#include <bwi_guidance_solver/mrn/abstract_mapping.h>

namespace bwi_guidance_solver {

  namespace mrn {
    
    void AbstractMapping::map(ExtendedState &state) {
      BOOST_FOREACH(RobotState &r, state.robots) {
        if (r.loc_p == 0.0f) {
          r.loc_v = r.loc_u;
          r.loc_p = 1.0f;
        }
        if (r.loc_p == 1.0f) {
          r.loc_u = r.loc_v;
        }
        if (r.loc_p != 1.0f) {
          r.loc_u = 0;
          r.loc_v = 0;
          r.loc_p = 0.5f;
        }
        if (r.help_destination == NONE && r.loc_v == r.tau_d && r.loc_p == 1.0f) {
          r.tau_t = r.tau_total_task_time / 2;
        }
      }
    }

  } /* mrn */
  
} /* bwi_guidance_solver */


