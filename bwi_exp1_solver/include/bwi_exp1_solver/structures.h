#ifndef STRUCTURES_HDY3OBT2
#define STRUCTURES_HDY3OBT2

#include <stdint.h>
#include <cstddef>

namespace boost {
  namespace serialization {
    class access;
  }
}

namespace bwi_exp1 {

  enum ActionType {
    DO_NOTHING = 0,
    PLACE_ROBOT = 1,
    PLACE_FUTURE_ROBOT = 2
  };

  // Simple model stuff

  struct State {
    size_t graph_id; // ~100
    size_t direction; // 0 to NUM_DIRECTIONS - 1
    size_t num_robots_left; // 0 to MAX_ROBOTS
  };

  struct Action {
    Action();
    Action(ActionType a, size_t g);
    ActionType type;
    size_t graph_id; // with PLACE ROBOT, identifies the direction pointed to

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
      ar & type;
      ar & graph_id;
    }
  };

  bool operator<(const Action& l, const Action& r);

  // Simple model has a very clean tabular representation, use idx instead
  typedef uint32_t state_t;
  typedef uint32_t action_t;

  enum RobotStatus {
    NO_ROBOT = -1,
    NO_DIRECTION_ON_ROBOT = -2
  };

  // Model 2 - now with lookahead
  struct State2 {
    int graph_id; // ~50
    int direction; // ~8 
    int num_robots_left; // ~6
    int current_robot_direction; // ~8
    int next_robot_location; // ~10

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
      ar & graph_id;
      ar & direction;
      ar & num_robots_left;
      ar & current_robot_direction;
      ar & next_robot_location;
    }
  };

  // This can often be seen written as
  bool operator<(const State2& l, const State2& r); 
  bool operator==(const State2& l, const State2& r);

} /* bwi_exp1 */

#endif /* end of include guard: STRUCTURES_HDY3OBT2 */
