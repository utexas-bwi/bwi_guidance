#ifndef BWI_GUIDANCE_SOLVER_MRN_STRUCTURES_H
#define BWI_GUIDANCE_SOLVER_MRN_STRUCTURES_H

#include <stdint.h>
#include <cstddef>
#include <ostream>

#include <bwi_guidance_solver/common.h>
#include <bwi_mapper/graph.h>

namespace bwi_guidance_solver {

  namespace mrn {
    
    /* Actions - need to be named differently from previous ActionTypes*/
    enum ActionType {
      WAIT = 0,
      ASSIGN_ROBOT = 1,
      DIRECT_PERSON = 2,
      RELEASE_ROBOT = 3,
      LEAD_PERSON = 4
    };

    struct Action {
      Action();
      Action(ActionType a, int robot_id = 0, int node = 0);
      ActionType type;

      int robot_id; // with ASSIGN_ROBOT, identifies the robot
      int node; // with DIRECT_PERSON or LEAD_PERSON, identifies the direction the robot should guide/lead to.
    };

    bool operator==(const Action& l, const Action& r);
    bool operator<(const Action& l, const Action& r);
    std::ostream& operator<<(std::ostream& stream, const Action& a);

    /* States */
    struct RobotState {

      int loc_u;
      int loc_v;
      float loc_p;

      int tau_d;
      float tau_t;
      float tau_total_task_time;
      int tau_u;

      int help_destination;

    };
    bool operator<(const RobotState& l, const RobotState& r); 
    bool operator==(const RobotState& l, const RobotState& r);
    bool operator>(const RobotState& l, const RobotState& r); 

    inline size_t hash_value(const bwi_guidance_solver::mrn::RobotState &rs) {
      size_t seed = 0;
      boost::hash_combine(seed, rs.loc_u);
      boost::hash_combine(seed, rs.loc_v);
      boost::hash_combine(seed, rs.loc_p);
      boost::hash_combine(seed, rs.tau_d);
      boost::hash_combine(seed, rs.tau_t);
      boost::hash_combine(seed, rs.tau_total_task_time);
      boost::hash_combine(seed, rs.tau_u);
      boost::hash_combine(seed, rs.help_destination);
      return seed;
    }

    struct State {

      int loc_node;
      int loc_prev; 

      int assist_type;
      int assist_loc;

      std::vector<RobotState> robots;

      /* The location precision is only used for visualization purposes. */
      float loc_p;
    };

    bool operator<(const State& l, const State& r); 
    bool operator==(const State& l, const State& r);
    std::ostream& operator<<(std::ostream& stream, const State& s);

  } /* mrn */
  
} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_STRUCTURES_H */
