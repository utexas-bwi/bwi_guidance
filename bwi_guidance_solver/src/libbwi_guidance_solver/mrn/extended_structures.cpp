#include <bwi_guidance_solver/mrn/extended_structures.h>

#define COMPARE(x) if((l.x) < (r.x)) return true; \
                   if((l.x) > (r.x)) return false;

namespace bwi_guidance_solver {

  namespace mrn {
    
    bool operator<(const ExtendedState& l, const ExtendedState& r) {
      COMPARE(prev_action);
      COMPARE(released_locations.size());
      // Then check if the vector contents are different.
      for (unsigned int i = 0; i < l.released_locations.size(); ++i) {
        COMPARE(released_locations[i]);
      }
      return ((const State)l < (const State)r);
    } 

    bool operator==(const ExtendedState& l, const ExtendedState& r) {
      return ((l.prev_action == r.prev_action) &&
              (l.released_locations == r.released_locations) && 
              ((const State)l == (const State)r));
    }

  } /* mrn */

} /* bwi_guidance_solver */

#undef COMPARE
