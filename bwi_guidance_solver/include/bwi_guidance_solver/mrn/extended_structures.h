#ifndef BWI_GUIDANCE_SOLVER_MRN_EXTENDED_STRUCTURE_H
#define BWI_GUIDANCE_SOLVER_MRN_EXTENDED_STRUCTURE_H

#include <bwi_guidance_solver/mrn/structures.h>

namespace bwi_guidance_solver {

  namespace mrn {

    struct ExtendedState : public State {
      Action prev_action;
      /* Locations that have been released since the last Wait action. No robot can be assigned to these locations. */
      std::vector<int> released_locations;
      std::vector<int> robot_provided_help;
    };

    bool operator<(const ExtendedState& l, const ExtendedState& r); 
    bool operator==(const ExtendedState& l, const ExtendedState& r);

    struct ExtendedStateHash { 
      ExtendedStateHash() {}
      size_t operator()(const ExtendedState& key) const {
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
        boost::hash_range(seed, key.robot_provided_help.begin(), key.robot_provided_help.end());
        /* Note that loc_v and precision are ignored here, as they are used for visualization purposes only. */
        return seed;
      }
    }; 

  } /* mrn */
  
} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_EXTENDED_STRUCTURE_H */
