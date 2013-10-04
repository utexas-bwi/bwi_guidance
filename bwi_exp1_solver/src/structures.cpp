#include <bwi_exp1_solver/structures.h>

namespace bwi_exp1 {

  Action::Action() : type(DO_NOTHING), graph_id(0) {}
  Action::Action(ActionType a, size_t g) : type(a), graph_id(g) {}

  bool operator<(const Action& l, const Action& r) {
    return (l.type < r.type) ||
      ((l.type == r.type) && (l.graph_id < r.graph_id)); 
  }

  std::ostream& operator<<(std::ostream& stream, const Action& a) {
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

  bool operator<(const State2& l, const State2& r ) {
    if (l.graph_id < r.graph_id) return true;
    if (l.graph_id > r.graph_id) return false;

    if (l.direction < r.direction) return true;
    if (l.direction > r.direction) return false;

    if (l.num_robots_left < r.num_robots_left) return true;
    if (l.num_robots_left > r.num_robots_left) return false;

    if (l.current_robot_status < r.current_robot_status) return true;
    if (l.current_robot_status > r.current_robot_status) return false;

    if (l.visible_robot_location < r.visible_robot_location) return true;
    if (l.visible_robot_location > r.visible_robot_location) return false;

    return false;
  }

  bool operator==(const State2& l, const State2& r ) {
    return (l.graph_id == r.graph_id &&
        l.direction == r.direction &&
        l.num_robots_left == r.num_robots_left &&
        l.current_robot_status == r.current_robot_status &&
        l.visible_robot_location == r.visible_robot_location);
  }

  std::ostream& operator<<(std::ostream& stream, const State2& s) {
    stream << "[" << s.graph_id << ", " 
        << s.direction << ", " 
        << s.num_robots_left << ", " 
        << s.current_robot_status << ", " << 
        s.visible_robot_location << "]";
    return stream;
  }

  size_t PersonModel2::computeNextDirection(size_t dir, size_t graph_id, 
      size_t next_graph_id) {
    float angle = getNodeAngle(graph_id, next_graph_id);
    return getDirectionFromAngle(angle);
  }

  size_t PersonModel2::getDirectionFromAngle(float angle) {
    angle = angle + M_PI / num_directions_;
    while (angle < 0) angle += 2 * M_PI;
    while (angle >= 2 * M_PI) angle -= 2 * M_PI;
    return (angle * num_directions_) / (2 * M_PI);
  }

  float PersonModel2::getAngleFromDirection(size_t dir) {
    return ((2 * M_PI) / num_directions_) * dir;
  }

} /* bwi_exp1 */
