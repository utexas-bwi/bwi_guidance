#ifndef STRUCTURES_HDY3OBT2
#define STRUCTURES_HDY3OBT2

#include <stdint.h>
#include <cstddef>
#include <ostream>

#include <bwi_mapper/graph.h>

namespace boost {
  namespace serialization {
    class access;
  }
}

namespace bwi_guidance {
  
  /* Constants */

  const unsigned NUM_DIRECTIONS = 16;

  enum MDPConstants {
    NONE = -1,
    DIR_UNASSIGNED = -2
  };

  /* Actions */

  enum ActionType {
    DO_NOTHING = 0,
    DIRECT_PERSON = 1,
    PLACE_ROBOT = 2
  };

  struct ActionQRR14 {
    ActionQRR14();
    ActionQRR14(ActionType a, size_t g);
    ActionType type;
    size_t graph_id; // with PLACE ROBOT, identifies the direction pointed to

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
      ar & type;
      ar & graph_id;
    }
  };

  bool operator==(const ActionQRR14& l, const ActionQRR14& r);
  bool operator<(const ActionQRR14& l, const ActionQRR14& r);
  std::ostream& operator<<(std::ostream& stream, const ActionQRR14& a);

  /* States */

  struct StateQRR14 {
    int graph_id; // ~50
    int direction; // ~8 
    int num_robots_left; // ~6
    int robot_direction; // ~8
    int visible_robot; // ~10

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
      ar & graph_id;
      ar & direction;
      ar & num_robots_left;
      ar & robot_direction;
      ar & visible_robot;
    }
  };

  bool operator<(const StateQRR14& l, const StateQRR14& r); 
  bool operator==(const StateQRR14& l, const StateQRR14& r);
  std::ostream& operator<<(std::ostream& stream, const StateQRR14& s);

  /* Helper Functions */

  size_t computeNextDirection(size_t dir, size_t graph_id, size_t
      next_graph_id, const bwi_mapper::Graph& graph);
  size_t getDiscretizedAngle(float angle);
  float getAngleInRadians(size_t dir);
  float getAbsoluteAngleDifference(float angle1, float angle2);

} /* bwi_guidance */

#endif /* end of include guard: STRUCTURES_HDY3OBT2 */
