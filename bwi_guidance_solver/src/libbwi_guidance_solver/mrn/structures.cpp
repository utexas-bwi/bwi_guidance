#include <bwi_guidance_solver/mrn/structures.h>

#define COMPARE(x) if((l.x) < (r.x)) return true; \
                   if((l.x) > (r.x)) return false;

namespace bwi_guidance_solver {

  namespace mrn {
    
    Action::Action() : type(WAIT), robot_id(0), node(0) {}
    Action::Action(ActionType a, int robot_id, int node) : type(a), robot_id(robot_id), node(node) {}

    bool operator==(const Action& l, const Action& r) {
      return ((l.type == r.type) && (l.robot_id == r.robot_id) && (l.node == r.node));
    }

    bool operator<(const Action& l, const Action& r) {
      return ((l.type < r.type) || 
              ((l.type == r.type) && (l.robot_id < r.robot_id)) || 
              ((l.type == r.type) && (l.robot_id == r.robot_id) && (l.node < r.node))); 
    }

    const std::string ACTION__NAMES[] = {
      "WAIT",
      "ASSIGN_ROBOT",
      "DIRECT_PERSON",
      "RELEASE_ROBOT",
      "LEAD_PERSON"
    };
    std::ostream& operator<<(std::ostream& stream, const Action& a) {
      stream << "[" << ACTION__NAMES[a.type];
      if (a.type != WAIT) {
        stream << " " << a.robot_id;
        if (a.type != RELEASE_ROBOT) {
        stream << "->" << a.node;
        }
      }
      stream << "]";
      return stream;
    }

    bool operator<(const RobotState& l, const RobotState& r) {
      COMPARE(loc_u);
      COMPARE(loc_v);
      COMPARE(loc_p);
      COMPARE(tau_d);
      COMPARE(tau_t);
      COMPARE(tau_u);
      COMPARE(help_destination);
      return false;
    }

    bool operator==(const RobotState& l, const RobotState& r) {
      return ((l.loc_u == r.loc_u) && 
              (l.loc_v == r.loc_v) &&
              (l.loc_p == r.loc_p) && 
              (l.tau_d == r.tau_d) &&
              (l.tau_t == r.tau_t) &&
              (l.tau_u == r.tau_u) &&
              (l.help_destination == r.help_destination));
    }

    bool operator<(const State& l, const State& r ) {
      COMPARE(loc_node);
      COMPARE(direction);
      COMPARE(assist_type);
      COMPARE(assist_loc);

      // First check size (should never be different).
      COMPARE(robots.size())

      // Then check if the vector contents are different.
      for (unsigned int i = 0; i < l.robots.size(); ++i) {
        COMPARE(robots[i]);
      }

      return false;
    }

    bool operator==(const State& l, const State& r ) {
      return l.loc_node == r.loc_node &&
        l.direction == r.direction &&
        l.assist_type == r.assist_type &&
        l.assist_loc == r.assist_loc &&
        l.robots == r.robots;
    }

    std::ostream& operator<<(std::ostream& stream, const State& s) {
      stream << "[" << s.loc_node << "," << s.direction << ",";
      for (unsigned int i = 0; i < s.robots.size(); ++i) {
        stream << "(<" << s.robots[i].loc_u << "," << s.robots[i].loc_v << "," << s.robots[i].loc_p << ">";
        stream << ",<" << s.robots[i].tau_d << "," << s.robots[i].tau_t << "," << s.robots[i].tau_u << ">";
        stream << "," << s.robots[i].help_destination << ")";
        stream << ","; 
      }
      stream << "),";
      stream << "<"  << s.assist_type << "," << s.assist_loc << ">]";
      return stream;
    }

  } /* mrn */

} /* bwi_guidance_solver */

#undef COMPARE
