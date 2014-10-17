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
      GUIDE_PERSON = 2,
      RELEASE_ROBOT = 3
    };

    struct Action {
      Action();
      Action(ActionType a, int dest, int dir);
      ActionType type;
      int at_graph_id; // with ASSIGN_ROBOT, identifies the destination
      int guide_graph_id; // with ASSIGN_ROBOT, identifies the direction the robot should point in

      friend class boost::serialization::access;
      template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
          ar & type;
          ar & at_graph_id;
          ar & guide_graph_id;
        }
    };

    bool operator==(const Action& l, const Action& r);
    bool operator<(const Action& l, const Action& r);
    std::ostream& operator<<(std::ostream& stream, const Action& a);

    /* States */
    struct RobotState {
      int graph_id; //~50
      int destination; //~50
      float precision; // value from 0 to 1, < 0.5 => graph_id -> other_graph_node, > 0.5 => other_graph_node -> graph_id
      int other_graph_node;

      friend class boost::serialization::access;
      template<class Archive>
        void serialize(Archive & ar, const unsigned int version) {
          ar & graph_id;
          ar & destination;
        }
    };
    bool operator<(const RobotState& l, const RobotState& r); 
    bool operator==(const RobotState& l, const RobotState& r);
    bool operator>(const RobotState& l, const RobotState& r); 

    inline size_t hash_value(const bwi_guidance_solver::mrn::RobotState &rs) {
      size_t seed = 0;
      boost::hash_combine(seed, rs.graph_id);
      boost::hash_combine(seed, rs.destination);
      boost::hash_combine(seed, rs.precision);
      boost::hash_combine(seed, rs.other_graph_node);
      return seed;
    }

    struct InUseRobotState {
      int robot_id; //~10
      int destination; //~20
      int direction; //~5
      bool reached_destination; //~2
      friend class boost::serialization::access;
      template<class Archive>
        void serialize(Archive & ar, const unsigned int version) {
          ar & robot_id;
          ar & destination;
          ar & direction;
        }
    };
    bool operator<(const InUseRobotState& l, const InUseRobotState& r); 
    bool operator==(const InUseRobotState& l, const InUseRobotState& r);
    bool operator>(const InUseRobotState& l, const InUseRobotState& r); 

    inline size_t hash_value(const bwi_guidance_solver::mrn::InUseRobotState &iurs) {
      size_t seed = 0;
      boost::hash_combine(seed, iurs.robot_id);
      boost::hash_combine(seed, iurs.destination);
      boost::hash_combine(seed, iurs.direction);
      boost::hash_combine(seed, iurs.reached_destination);
      return seed;
    }

    struct State {
      int graph_id; // ~50
      int direction; // ~8 

      /* Ignored while in the map */
      float precision;
      int from_graph_node;

      bool robot_gave_direction;

      std::vector<RobotState> robots; // ~10 * 50 * 50
      std::vector<InUseRobotState> in_use_robots; // ~10 * 20 * 5

      /* These just prevent bad action choices */
      std::vector<int> acquired_locations;
      std::vector<int> relieved_locations;

      friend class boost::serialization::access;
      template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
          ar & graph_id;
          ar & direction;
          ar & BOOST_SERIALIZATION_NVP(robots);
          ar & BOOST_SERIALIZATION_NVP(in_use_robots);
        }
    };

    struct StateHash { 
      StateHash() {}
      size_t operator()(const State& key) const {
        size_t seed = 0;
        boost::hash_combine(seed, key.graph_id);
        boost::hash_combine(seed, key.direction);
        /* boost::hash_combine(seed, key.precision); */
        /* boost::hash_combine(seed, key.from_graph_node); */
        boost::hash_combine(seed, key.robot_gave_direction);
        /* boost::hash_range(seed, key.robots.begin(), key.robots.end()); */
        boost::hash_range(seed, key.in_use_robots.begin(), key.in_use_robots.end());
        boost::hash_range(seed, key.acquired_locations.begin(), key.acquired_locations.end());
        return seed;
      }
    }; 

    bool operator<(const State& l, const State& r); 
    bool operator==(const State& l, const State& r);
    std::ostream& operator<<(std::ostream& stream, const State& s);

  } /* mrn */
  
} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_STRUCTURES_H */
