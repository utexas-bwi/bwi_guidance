#include <bwi_guidance_solver/mrn/common.h>

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

      float current_edge_distance = shortest_distances[loc_u][loc_v];
      float distance_to_u = loc_p * current_edge_distance;
      float distance_to_v = current_edge_distance - distance_to_u;

      float distance_from_u = shortest_distances[loc_u][destination] + distance_to_u;
      float distance_from_v = shortest_distances[loc_v][destination] + distance_to_v;
      shortest_path_through_u = (distance_from_u < distance_from_v);

      return (shortest_path_through_u) ? distance_from_u : distance_from_v;
    }
    
    int selectBestRobotForTask(const State& state, 
                               int destination, 
                               float human_speed, 
                               float robot_speed,
                               const std::vector<std::vector<float> > &shortest_distances,
                               bool &reach_in_time) {

      std::vector<float> utility_loss(state.robots.size(), std::numeric_limits<float>::max());
      std::vector<float> time(state.robots.size(), std::numeric_limits<float>::max());

      float time_to_destination = shortest_distances[state.loc_node][destination] / human_speed;

      for (int i = 0; i < state.robots.size(); ++i) {
        if (state.robots[i].help_destination != NONE) {
          // This robot is already helping the human, and cannot be used.
          continue;
        }

        // Calculate this robot's time to its current destination.
        float orig_distance = getTrueDistanceTo(state.robots[i].loc_u,
                                                state.robots[i].loc_v,
                                                state.robots[i].loc_p,
                                                state.robots[i].tau_d,
                                                shortest_distances);
        float original_time = orig_distance / robot_speed;

        // Calculate time if it is diverted to given location.
        float new_distance_1 = getTrueDistanceTo(state.robots[i].loc_u,
                                                 state.robots[i].loc_v,
                                                 state.robots[i].loc_p,
                                                 destination,
                                                 shortest_distances);
        float new_distance_2 = shortest_distances[destination][state.robots[i].tau_d];
        float new_time = 
          std::max(new_distance_1 / robot_speed, time_to_destination) + 
          new_distance_2 / robot_speed;
        if (new_distance_1 / robot_speed <= time_to_destination) {
          // The robot will reach there in time
          utility_loss[i] = state.robots[i].tau_u * (new_time - original_time);
        }
        time[i] = new_distance_1 / robot_speed;
      }

      if (*(std::min_element(utility_loss.begin(), utility_loss.end())) != std::numeric_limits<float>::max()) {
        reach_in_time = true;
        return std::min_element(utility_loss.begin(), utility_loss.end()) - utility_loss.begin();
      } 
      reach_in_time = false;
      return std::min_element(time.begin(), time.end()) - time.begin();

    }

  } /* mrn */
  
} /* bwi_guidance_solver */
