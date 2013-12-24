#ifndef ROBOTS_OKE3LSAK
#define ROBOTS_OKE3LSAK

#include <string>
#include <vector>

#include <bwi_guidance/structures.h>

namespace bwi_guidance {

  struct DefaultRobots {
    std::vector<Robot> robots;
  };

  void readDefaultRobotsFromFile(const std::string& robot_file, 
      DefaultRobots& default_robots);

  struct DCInstanceRobots {
    std::vector<PathPoint> path;
    std::vector<Location> robots;
  };

  struct DCInstanceGroupRobots {
    std::string prefix;
    std::vector<DCInstanceRobots> instances;
  };

  struct DCExperimentRobots {
    std::vector<DCInstanceGroupRobots> instance_groups;
  };

  void readDCExperimentRobotsFromFile(const std::string& robot_file,
      DCExperimentRobots& er);

  const DCInstanceRobots& getDCInstance(const std::string& instance_name,
      const DCExperimentRobots& er);
  
} /* bwi_guidance */

#endif /* end of include guard: ROBOTS_OKE3LSAK */
