#ifndef BWI_GUIDANCE_SOLVER_MRN_RESTRICTED_STRUCTURES_H
#define BWI_GUIDANCE_SOLVER_MRN_RESTRICTED_STRUCTURES_H

#include <bwi_guidance_solver/mrn/structures.h>

namespace bwi_guidance_solver {

  namespace mrn {

    struct RestrictedAction : public Action {
      RestrictedAction(ActionType a = WAIT, int arg1 = 0) : Action(a, arg1, arg1) {}
    };
    
    std::ostream& operator<<(std::ostream& stream, const RestrictedAction& a);

    struct RestrictedState : public State {
      RestrictedAction prev_action;
      /* Locations that have been released since the last Wait action. No robot can be assigned to these locations. */
      std::vector<int> released_locations;
    };

    bool operator<(const RestrictedState& l, const RestrictedState& r); 
    bool operator==(const RestrictedState& l, const RestrictedState& r);

    struct RestrictedStateHash { 
      RestrictedStateHash() {}
      size_t operator()(const RestrictedState& key) const {
        size_t seed = 0;
        boost::hash_combine(seed, key.loc_node);
        boost::hash_combine(seed, key.loc_prev);
        boost::hash_combine(seed, key.assist_type);
        boost::hash_combine(seed, key.assist_loc);
        boost::hash_combine(seed, key.prev_action.type);
        boost::hash_combine(seed, key.prev_action.robot_id);
        boost::hash_combine(seed, key.prev_action.node);
        boost::hash_range(seed, key.robots.begin(), key.robots.end());
        boost::hash_range(seed, key.released_locations.begin(), key.released_locations.end());
        /* Note that loc_v and precision are ignored here, as they are used for visualization purposes only. */
        return seed;
      }
    }; 

  } /* mrn */
  
} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_RESTRICTED_STRUCTURES_H */
