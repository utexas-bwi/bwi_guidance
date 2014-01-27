#ifndef BWI_GUIDANCE_SOLVER_STRUCTURES_IROS14
#define BWI_GUIDANCE_SOLVER_STRUCTURES_IROS14

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

namespace bwi_guidance {
  
  /* Actions */
  enum ActionTypeIROS14 {
    DO_NOTHING = 0,
    ASSIGN_ROBOT = 1,
    GUIDE_PERSON = 2,
    RELEASE_ROBOT = 3
  };

  struct ActionIROS14 {
    ActionIROS14();
    ActionIROS14(ActionTypeIROS14 a, int dest, int dir);
    ActionTypeIROS14 type;
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

  bool operator==(const ActionIROS14& l, const ActionIROS14& r);
  bool operator<(const ActionIROS14& l, const ActionIROS14& r);
  std::ostream& operator<<(std::ostream& stream, const ActionIROS14& a);

  /* States */
  struct RobotStateIROS14 {
    int graph_id; //~50
    int destination; //~50

    /* Ignored while being kept in a map, the extra precision is lost there.
     * Don't use these members in operator<, == or > overloading */
    float precision; // value from -0.5 to 0.5
    int from_graph_node;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
      ar & graph_id;
      ar & destination;
    }
  };
  bool operator<(const RobotStateIROS14& l, const RobotStateIROS14& r); 
  bool operator==(const RobotStateIROS14& l, const RobotStateIROS14& r);
  bool operator>(const RobotStateIROS14& l, const RobotStateIROS14& r); 
  
  struct InUseRobotStateIROS14 {
    int robot_id; //~10
    int destination; //~20
    int direction; //~5
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
      ar & robot_id;
      ar & destination;
      ar & direction;
    }
  };
  bool operator<(const InUseRobotStateIROS14& l, const InUseRobotStateIROS14& r); 
  bool operator==(const InUseRobotStateIROS14& l, const InUseRobotStateIROS14& r);
  bool operator>(const InUseRobotStateIROS14& l, const InUseRobotStateIROS14& r); 

  struct StateIROS14 {
    int graph_id; // ~50
    int direction; // ~8 
    std::vector<RobotStateIROS14> robots; // ~10 * 50 * 50
    std::vector<InUseRobotStateIROS14> in_use_robots; // ~10 * 20 * 5

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

  bool operator<(const StateIROS14& l, const StateIROS14& r); 
  bool operator==(const StateIROS14& l, const StateIROS14& r);
  std::ostream& operator<<(std::ostream& stream, const StateIROS14& s);

} /* bwi_guidance */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_STRUCTURES_IROS14 */
