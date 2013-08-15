#ifndef ROBOTS_OKE3LSAK
#define ROBOTS_OKE3LSAK

#include <bwi_exp1/structures.h>

namespace bwi_exp1 {

  struct InstanceRobots {
    std::vector<PathPoint> path;
    std::vector<Location> robots;
  };

  struct ExperimentRobots {
    std::vector<Robot> robots;
    std::vector<InstanceRobots> instances; 
  };
  
} /* bwi_exp1 */

#endif /* end of include guard: ROBOTS_OKE3LSAK */
