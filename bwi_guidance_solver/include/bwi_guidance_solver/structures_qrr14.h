#ifndef STRUCTURES_HDY3OBT2
#define STRUCTURES_HDY3OBT2

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

} /* bwi_guidance */

#endif /* end of include guard: STRUCTURES_HDY3OBT2 */
