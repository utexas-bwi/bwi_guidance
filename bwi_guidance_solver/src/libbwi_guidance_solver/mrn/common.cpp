#include <bwi_guidance_solver/mrn/common.cpp>

namespace bwi_guidance_solver {

  namespace mrn {

    bool isShortestPathThroughLocU(int loc_u, 
                                   int loc_v, 
                                   float loc_p, 
                                   int destination, 
                                   const std::vector<std::vector<float> > &shortest_distances) {
      bool ret_val;
      getTrueDistanceTo(loc_u, loc_v, loc_p, destination, shortest_distances, ret_val);
      return ret_val;
    }

    float getTrueDistanceTo(int loc_u, 
                            int loc_v, 
                            float loc_p, 
                            int destination, 
                            const std::vector<std::vector<float> > &shortest_distances) {
      bool unused_ret_val;
      return getTrueDistanceTo(loc_u, loc_v, loc_p, destination, shortest_distances, unused_ret_val);
    }
    
    float getTrueDistanceTo(int loc_u, 
                            int loc_v, 
                            float loc_p, 
                            int destination, 
                            const std::vector<std::vector<float> > &shortest_distances,
                            bool &shortest_path_through_u) {
      float current_edge_distance = shortest_distances_[loc_u][loc_v];
      float distance_to_u = loc_p * current_edge_distance;
      float distance_to_v = current_edge_distance - distance_to_u;

      float distance_from_u = shortest_distance_[loc_u][destination] + distance_to_u;
      float distance_from_v = shortest_distance_[loc_u][destination] + distance_to_u;
      shortest_path_through_u = (distance_from_u < distance_from_v);

      return (shortest_path_through_u) ? distance_from_u : distance_from_v;
    }
    
  } /* mrn */
  
} /* bwi_guidance_solver */
