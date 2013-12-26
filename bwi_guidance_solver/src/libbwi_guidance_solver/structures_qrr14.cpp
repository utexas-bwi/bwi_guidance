#include <bwi_guidance_solver/structures_qrr14.h>
#include <bwi_mapper/graph.h>

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

  size_t computeNextDirection(size_t dir, size_t graph_id, 
      size_t next_graph_id, const bwi_mapper::Graph& graph) {
    float angle = 
      bwi_mapper::getNodeAngle(graph_id, next_graph_id, graph);
    return getDiscretizedAngle(angle);
  }

  size_t getDiscretizedAngle(float angle) {
    angle = angle + M_PI / NUM_DIRECTIONS;
    while (angle < 0) angle += 2 * M_PI;
    while (angle >= 2 * M_PI) angle -= 2 * M_PI;
    return (angle * NUM_DIRECTIONS) / (2 * M_PI);
  }

  float getAngleInRadians(size_t dir) {
    return ((2 * M_PI) / NUM_DIRECTIONS) * dir;
  }

  float getAbsoluteAngleDifference(float angle1, float angle2) {
    while (angle2 > angle1 + M_PI) angle2 -= 2 * M_PI;
    while (angle2 <= angle1 - M_PI) angle2 += 2 * M_PI;
    return fabs (angle1 - angle2);
  }

} /* bwi_guidance */
