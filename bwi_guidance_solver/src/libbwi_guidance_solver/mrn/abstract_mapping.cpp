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
          r.loc_p = 0.5f;
          r.loc_v = 0;
        }
        if (r.tau_t != 0.0f && r.tau_t != 1.0f) {
          r.tau_t = 15.0f;//r.tau_total_task_time / 2;
        }
      }
    }

  } /* mrn */

} /* bwi_guidance_solver */


