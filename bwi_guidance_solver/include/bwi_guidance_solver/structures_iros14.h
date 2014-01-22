#ifndef BWI_GUIDANCE_SOLVER_STRUCTURES_IROS14
#define BWI_GUIDANCE_SOLVER_STRUCTURES_IROS14

#include <stdint.h>
#include <cstddef>
#include <ostream>

#include <bwi_guidance_solver/structures_qrr14.h>
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
    DIRECT_ROBOT = 1,
    RELEASE_ROBOT = 2
  };

  struct ActionIROS14 {
    ActionIROS14();
    ActionIROS14(ActionTypeIROS14 a, int destination, int graph_id);
    ActionTypeIROS14 type;
    size_t graph_id; // with PLACE ROBOT, identifies the direction pointed to

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
      ar & type;
      ar & graph_id;
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

  /* Helper Functions */

  size_t computeNextDirection(size_t dir, size_t graph_id, size_t
      next_graph_id, const bwi_mapper::Graph& graph);
  size_t getDiscretizedAngle(float angle);
  float getAngleInRadians(size_t dir);
  float getAbsoluteAngleDifference(float angle1, float angle2);

} /* bwi_guidance */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_STRUCTURES_IROS14 */
