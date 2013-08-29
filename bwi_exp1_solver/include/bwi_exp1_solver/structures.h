#ifndef STRUCTURES_HDY3OBT2
#define STRUCTURES_HDY3OBT2

#include <stdint.h>

namespace bwi_exp1 {

  struct State {
    size_t graph_id; // ~100
    size_t direction; // 0 to NUM_DIRECTIONS - 1
    size_t num_robots_left; // 0 to MAX_ROBOTS
  };

  enum ActionType {
    DO_NOTHING = 0,
    PLACE_ROBOT = 1,
    PLACE_FUTURE_ROBOT = 2
  };

  class Action {
    public:
      Action();
      Action(ActionType a, size_t g);
      ActionType type;
      size_t graph_id; // with PLACE ROBOT, identifies the direction pointed to
  };

  struct State2 {
    size_t graph_id; // ~100
    size_t direction;
    bool robot_at_current;
    size_t current_robot_direction;

  class Action2 {
    public:
      Action2();
      Action2(ActionType a, size_t g);
      ActionType type;
      size_t graph_id;
  };

  typedef uint32_t state_t;
  typedef uint32_t action_t;

} /* bwi_exp1 */

#endif /* end of include guard: STRUCTURES_HDY3OBT2 */
