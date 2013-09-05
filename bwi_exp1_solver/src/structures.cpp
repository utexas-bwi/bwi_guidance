#include <bwi_exp1_solver/structures.h>

namespace bwi_exp1 {

  Action::Action() : type(DO_NOTHING), graph_id(0) {}
  Action::Action(ActionType a, size_t g) : type(a), graph_id(g) {}

  bool operator<(const Action& l, const Action& r) {
    return (l.type < r.type) ||
      ((l.type == r.type) && (l.graph_id < r.graph_id)); 
  }

  bool operator<(const State2& l, const State2& r ) {
    if (l.graph_id < r.graph_id) return true;
    if (l.graph_id > r.graph_id) return false;

    if (l.direction < r.direction) return true;
    if (l.direction > r.direction) return false;

    if (l.num_robots_left < r.num_robots_left) return true;
    if (l.num_robots_left > r.num_robots_left) return false;

    if (l.current_robot_direction < r.current_robot_direction) return true;
    if (l.current_robot_direction > r.current_robot_direction) return false;

    if (l.next_robot_location < r.next_robot_location) return true;
    if (l.next_robot_location > r.next_robot_location) return false;

    return false;
  }

  bool operator==(const State2& l, const State2& r ) {
    return (l.graph_id == r.graph_id &&
        l.direction == r.direction &&
        l.num_robots_left == r.num_robots_left &&
        l.current_robot_direction == r.current_robot_direction &&
        l.next_robot_location == r.next_robot_location);
  }

} /* bwi_exp1 */
