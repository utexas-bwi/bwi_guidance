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
    RELEASE_ROBOT = 2
  };

  struct ActionIROS14 {
    ActionIROS14();
    ActionIROS14(ActionTypeIROS14 a, int at_graph_id, int guide_graph_id);
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
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
      ar & graph_id;
      ar & destination;
    }
  };

  struct StateIROS14 {
    int graph_id; // ~50
    int direction; // ~8 
    std::vector<RobotStateIROS14> robots; // ~10 * 50 * 50
    int robot_destination; // ~20
    int robot_direction; // ~5

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
      ar & graph_id;
      ar & direction;
      ar & BOOST_SERIALIZATION_NVP(robots);
      ar & robot_destination;
      ar & robot_direction;
    }
  };

  bool operator<(const StateIROS14& l, const StateIROS14& r); 
  bool operator==(const StateIROS14& l, const StateIROS14& r);
  std::ostream& operator<<(std::ostream& stream, const StateIROS14& s);

} /* bwi_guidance */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_STRUCTURES_IROS14 */
