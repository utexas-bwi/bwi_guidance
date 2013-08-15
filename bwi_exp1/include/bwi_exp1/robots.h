#ifndef ROBOTS_OKE3LSAK
#define ROBOTS_OKE3LSAK

#include <string>
#include <vector>

#include <bwi_exp1/structures.h>

namespace bwi_exp1 {

  struct InstanceRobots {
    std::vector<PathPoint> path;
    std::vector<Location> robots;
  };

  struct InstanceGroupRobots {
    std::string prefix;
    std::vector<InstanceRobots> instances;
  };

  struct ExperimentRobots {
    std::vector<Robot> robots;
    std::vector<InstanceGroupRobots> instance_groups;
  };

  void readRobotsFromFile(const std::string& robot_file, 
      ExperimentRobots& experiment_robots_);

  const InstanceRobots& getInstance(
      const ExperimentRobots& er, std::string& instance_name);
  
} /* bwi_exp1 */

#endif /* end of include guard: ROBOTS_OKE3LSAK */
