#ifndef BWI_GUIDANCE_SOLVER_MRN_STRUCTURES_H
#define BWI_GUIDANCE_SOLVER_MRN_STRUCTURES_H

#include <stdint.h>
#include <cstddef>
#include <ostream>

#include <bwi_guidance_solver/common.h>
#include <bwi_mapper/graph.h>

namespace boost {
  namespace serialization {
    class access;
  }
}

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

      friend class boost::serialization::access;
      template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
          ar & type;
          ar & robot_id;
          ar & node;
        }
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
      int tau_t;
      int tau_u;

      int help_destination;

      friend class boost::serialization::access;
      template<class Archive>
        void serialize(Archive & ar, const unsigned int version) {
          ar & node;
          ar & destination;
        }
    };
    bool operator<(const RobotState& l, const RobotState& r); 
    bool operator==(const RobotState& l, const RobotState& r);

    inline size_t hash_value(const bwi_guidance_solver::mrn::RobotState &rs) {
      size_t seed = 0;
      boost::hash_combine(seed, rs.loc_u);
      boost::hash_combine(seed, rs.loc_v);
      boost::hash_combine(seed, rs.loc_p);
      boost::hash_combine(seed, rs.tau_d);
      boost::hash_combine(seed, rs.tau_t);
      boost::hash_combine(seed, rs.tau_u);
      boost::hash_combine(seed, rs.help_destination);
      return seed;
    }

    struct State {

      int loc_node; // ~50
      int direction; // ~16 

      int assist_type;
      int assist_loc;

      std::vector<RobotState> robots;

      /* These are only used for visualization purposes. loc_node is the same as loc_u. */
      int loc_v;
      float loc_p;

      friend class boost::serialization::access;
      template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
          ar & loc_node;
          ar & direction;
          ar & assist_type;
          ar & assist_loc;
          ar & BOOST_SERIALIZATION_NVP(robots);
          ar & loc_v;
          ar & loc_p;
        }
    };

    struct StateHash { 
      StateHash() {}
      size_t operator()(const State& key) const {
        size_t seed = 0;
        boost::hash_combine(seed, key.loc_node);
        boost::hash_combine(seed, key.direction);
        boost::hash_combine(seed, key.assist_type);
        boost::hash_combine(seed, key.assist_loc);
        boost::hash_range(seed, key.robots.begin(), key.robots.end());
        /* Note that loc_v and precision are ignored here, as they are used for visualization purposes only. */
        return seed;
      }
    }; 

    bool operator<(const State& l, const State& r); 
    bool operator==(const State& l, const State& r);
    std::ostream& operator<<(std::ostream& stream, const State& s);

  } /* mrn */
  
} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_STRUCTURES_H */
