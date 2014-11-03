#ifndef BWI_GUIDANCE_SOLVER_MRN_COMMON_H
#define BWI_GUIDANCE_SOLVER_MRN_COMMON_H

#include <bwi_guidance_solver/mrn/structures.h>
#include <fstream>
#include <vector>

namespace bwi_guidance_solver {

  namespace mrn {

    bool isShortestPathThroughLocU(int loc_u, 
                                   int loc_v, 
                                   float loc_p, 
                                   int destination, 
                                   const std::vector<std::vector<float> > &shortest_distances);

    float getTrueDistanceTo(int loc_u, 
                            int loc_v, 
                            float loc_p, 
                            int destination, 
                            const std::vector<std::vector<float> > &shortest_distances);
    
    float getTrueDistanceTo(int loc_u, 
                            int loc_v, 
                            float loc_p, 
                            int destination, 
                            const std::vector<std::vector<float> > &shortest_distances,
                            bool &shortest_path_through_u);

    inline bool isRobotExactlyAt(const RobotState& robot, int loc) {
      return (((robot.loc_u == loc) && (robot.loc_p == 0.0f)) ||
              ((robot.loc_v == loc) && (robot.loc_p == 1.0f)));
    }

    int selectBestRobotForTask(const State& state, 
                               int destination, 
                               float human_speed, 
                               float robot_speed,
                               const std::vector<std::vector<float> > &shortest_distances);

    inline void readRobotHomeBase(const std::string& robot_home_base_file, std::vector<int> &robot_home_base) {
      robot_home_base.clear();
      std::ifstream fin(robot_home_base_file.c_str());
      while (!fin.eof()) {
        int home_base;
        fin >> home_base;
        robot_home_base.push_back(home_base);
      }
      fin.close();
    }

  } /* mrn */
  
} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_COMMON_H */
