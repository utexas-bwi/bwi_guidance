#include <bwi_guidance_solver/mrn/restricted_structures.h>

#define COMPARE(x) if((l.x) < (r.x)) return true; \
                   if((l.x) > (r.x)) return false;

namespace bwi_guidance_solver {

  namespace mrn {
    
    RestrictedAction::RestrictedAction() : Action() {}
    RestrictedAction::RestrictedAction(ActionType a, int robot_id, int node) : Action(a, robot_id, node) {}

    const std::string ACTION_NAMES[] = {
      "WAIT",
      "ASSIGN_ROBOT",
      "DIRECT_PERSON",
      "RELEASE_ROBOT",
      "LEAD_PERSON"
    };
    std::ostream& operator<<(std::ostream& stream, const RestrictedAction& a) {
      stream << "[" << ACTION_NAMES[a.type];
      if (a.type != WAIT) {
        if (a.type == RELEASE_ROBOT) {
          stream << " " << a.robot_id;
        } else {
          stream << " " << a.node;
        }
      }
      stream << "]";
      return stream;
    }

    bool operator<(const RestrictedState& l, const RestrictedState& r) {
      COMPARE(prev_action);
      COMPARE(released_locations.size());
      // Then check if the vector contents are different.
      for (unsigned int i = 0; i < l.released_locations.size(); ++i) {
        COMPARE(released_locations[i]);
      }
      return ((const State)l < (const State)r);
    } 

    bool operator==(const RestrictedState& l, const RestrictedState& r) {
      return ((l.prev_action == r.prev_action) &&
              (l.released_locations == r.released_locations) && 
              ((const State)l == (const State)r));
    }

  } /* mrn */

} /* bwi_guidance_solver */

#undef COMPARE
