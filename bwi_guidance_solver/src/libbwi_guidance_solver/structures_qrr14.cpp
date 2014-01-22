#include <bwi_guidance_solver/structures_qrr14.h>

namespace bwi_guidance {


  ActionQRR14::ActionQRR14() : type(DO_NOTHING), graph_id(0) {}
  ActionQRR14::ActionQRR14(ActionType a, size_t g) : type(a), graph_id(g) {}

  bool operator==(const ActionQRR14& l, const ActionQRR14& r) {
    return (l.type == r.type) && (l.graph_id == r.graph_id);
  }

  bool operator<(const ActionQRR14& l, const ActionQRR14& r) {
    return (l.type < r.type) ||
      ((l.type == r.type) && (l.graph_id < r.graph_id)); 
  }

  std::ostream& operator<<(std::ostream& stream, const ActionQRR14& a) {
    std::string action_str[3];
    action_str[DO_NOTHING] = "DO_NOTHING";
    action_str[PLACE_ROBOT] = "PLACE_ROBOT";
    action_str[DIRECT_PERSON] = "DIRECT_PERSON";

    stream << "[" << action_str[a.type];
    if (a.type != DO_NOTHING)
      stream << " " << a.graph_id;
    stream << "]";

    return stream;
  }

  bool operator<(const StateQRR14& l, const StateQRR14& r ) {
    if (l.graph_id < r.graph_id) return true;
    if (l.graph_id > r.graph_id) return false;

    if (l.direction < r.direction) return true;
    if (l.direction > r.direction) return false;

    if (l.num_robots_left < r.num_robots_left) return true;
    if (l.num_robots_left > r.num_robots_left) return false;

    if (l.robot_direction < r.robot_direction) return true;
    if (l.robot_direction > r.robot_direction) return false;

    if (l.visible_robot < r.visible_robot) return true;
    if (l.visible_robot > r.visible_robot) return false;

    return false;
  }

  bool operator==(const StateQRR14& l, const StateQRR14& r ) {
    return (l.graph_id == r.graph_id &&
        l.direction == r.direction &&
        l.num_robots_left == r.num_robots_left &&
        l.robot_direction == r.robot_direction &&
        l.visible_robot == r.visible_robot);
  }

  std::ostream& operator<<(std::ostream& stream, const StateQRR14& s) {
    stream << "[" << s.graph_id << ", " 
        << s.direction << ", " 
        << s.num_robots_left << ", " 
        << s.robot_direction << ", " << 
        s.visible_robot << "]";
    return stream;
  }

} /* bwi_guidance */
