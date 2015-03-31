#include <bwi_guidance_solver/irm/structures.h>

namespace bwi_guidance_solver {

  namespace irm {

    Action::Action() : type(DO_NOTHING), graph_id(0) {}
    Action::Action(ActionType a, size_t g) : type(a), graph_id(g) {}

    bool operator==(const Action& l, const Action& r) {
      return (l.type == r.type) && (l.graph_id == r.graph_id);
    }

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

    bool operator<(const State& l, const State& r ) {
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

    bool operator==(const State& l, const State& r ) {
      return (l.graph_id == r.graph_id &&
              l.direction == r.direction &&
              l.num_robots_left == r.num_robots_left &&
              l.robot_direction == r.robot_direction &&
              l.visible_robot == r.visible_robot);
    }

    std::ostream& operator<<(std::ostream& stream, const State& s) {
      stream << "[" << s.graph_id << ", " 
        << s.direction << ", " 
        << s.num_robots_left << ", " 
        << s.robot_direction << ", " << 
        s.visible_robot << "]";
      return stream;
    }

  } /* irm - InstantaneousRobotMotion */

} /* bwi_guidance_solver */
