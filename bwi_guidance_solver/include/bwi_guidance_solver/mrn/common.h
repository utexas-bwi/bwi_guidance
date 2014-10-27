#ifndef BWI_GUIDANCE_SOLVER_MRN_COMMON_H
#define BWI_GUIDANCE_SOLVER_MRN_COMMON_H

#include <bwi_guidance_solver/mrn/structures.h>
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

  } /* mrn */
  
} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_COMMON_H */
