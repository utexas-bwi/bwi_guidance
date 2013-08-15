#ifndef LOCATION_5DFWRZLV
#define LOCATION_5DFWRZLV

#include <string>

namespace bwi_exp1 {

  struct Location {
    float x;
    float y;
    float yaw;
  };

  struct Robot {
    std::string id;
    Location default_loc;
  };

  struct PathPoint {
    size_t graph_id;
    bool robot_present;
  };

  struct LocationStamped {
    Location location;
    float seconds_since_start;
  };
  
} /* bwi_exp1 */


#endif /* end of include guard: LOCATION_5DFWRZLV */
