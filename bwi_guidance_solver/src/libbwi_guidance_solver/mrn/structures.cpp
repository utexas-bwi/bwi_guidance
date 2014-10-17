#include <bwi_guidance_solver/mrn/structures.h>

namespace bwi_guidance_solver {

  namespace mrn {
    
    Action::Action() : type(WAIT), at_graph_id(0), guide_graph_id(0) {}
    Action::Action(ActionType a, int dest, int dir) : type(a), at_graph_id(dest), guide_graph_id(dir) {}

    bool operator==(const Action& l, const Action& r) {
      return ((l.type == r.type) && (l.at_graph_id == r.at_graph_id) && (l.guide_graph_id == r.guide_graph_id));
    }

    bool operator<(const Action& l, const Action& r) {
      return ((l.type < r.type) ||
              ((l.type == r.type) && (l.at_graph_id < r.at_graph_id)) ||
              ((l.type == r.type) && (l.at_graph_id == r.at_graph_id) && (l.guide_graph_id < r.guide_graph_id))); 
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
      if (a.type != WAIT)
        stream << " " << a.at_graph_id << "->" << a.guide_graph_id;
      stream << "]";
      return stream;
    }

    bool operator<(const RobotState& l, const RobotState& r) {
      return (l.graph_id < r.graph_id) ||
        ((l.graph_id == r.graph_id) && (l.destination < r.destination)) ||
        ((l.graph_id == r.graph_id) && (l.destination == r.destination) &&
         (l.precision < r.precision)) ||
        ((l.graph_id == r.graph_id) && (l.destination == r.destination) &&
         (l.precision == r.precision) && 
         (l.other_graph_node < r.other_graph_node));
    }

    bool operator==(const RobotState& l, const RobotState& r) {
      return (l.graph_id == r.graph_id) && 
        (l.destination == r.destination) &&
        (l.precision == r.precision) && 
        (l.other_graph_node == r.other_graph_node); 
    }

    bool operator>(const RobotState& l, const RobotState& r) {
      return (l.graph_id > r.graph_id) ||
        ((l.graph_id == r.graph_id) && (l.destination > r.destination)) ||
        ((l.graph_id == r.graph_id) && (l.destination == r.destination) &&
         (l.precision > r.precision)) ||
        ((l.graph_id == r.graph_id) && (l.destination == r.destination) &&
         (l.precision == r.precision) && 
         (l.other_graph_node > r.other_graph_node));
    }

    bool operator<(const InUseRobotState& l, const InUseRobotState& r) {
      return (l.robot_id < r.robot_id) ||
        ((l.robot_id == r.robot_id) && (l.destination < r.destination)) ||
        ((l.robot_id == r.robot_id) && (l.destination == r.destination) &&
         (l.direction < r.direction)) ||
        ((l.robot_id == r.robot_id) && (l.destination == r.destination) &&
         (l.direction == r.direction) && 
         (l.reached_destination < r.reached_destination));
    }

    bool operator==(const InUseRobotState& l, const InUseRobotState& r) {
      return (l.robot_id == r.robot_id) && 
        (l.destination == r.destination) &&
        (l.direction == r.direction) && 
        (l.reached_destination == r.reached_destination); 
    }

    bool operator>(const InUseRobotState& l, const InUseRobotState& r) {
      return (l.robot_id > r.robot_id) ||
        ((l.robot_id == r.robot_id) && (l.destination > r.destination)) ||
        ((l.robot_id == r.robot_id) && (l.destination == r.destination) &&
         (l.direction > r.direction)) ||
        ((l.robot_id == r.robot_id) && (l.destination == r.destination) &&
         (l.direction == r.direction) && 
         (l.reached_destination > r.reached_destination));
    }

    bool operator<(const State& l, const State& r ) {
      if (l.graph_id < r.graph_id) return true;
      if (l.graph_id > r.graph_id) return false;

      if (l.direction < r.direction) return true;
      if (l.direction > r.direction) return false;

      if (l.robot_gave_direction < r.robot_gave_direction) return true;
      if (l.robot_gave_direction > r.robot_gave_direction) return false;

      if (l.precision < r.precision) return true;
      if (l.precision > r.precision) return false;

      if (l.from_graph_node < r.from_graph_node) return true;
      if (l.from_graph_node > r.from_graph_node) return false;

      // Vectors are checked in the order of least complex to most complex.
      // First check if any of the vectors are different length.
      if (l.acquired_locations.size() < r.acquired_locations.size()) return true;
      if (l.acquired_locations.size() > r.acquired_locations.size()) return false;

      if (l.relieved_locations.size() < r.relieved_locations.size()) return true;
      if (l.relieved_locations.size() > r.relieved_locations.size()) return false;

      if (l.in_use_robots.size() < r.in_use_robots.size()) return true;
      if (l.in_use_robots.size() > r.in_use_robots.size()) return false;

      if (l.robots.size() < r.robots.size()) return true;
      if (l.robots.size() > r.robots.size()) return false;

      // Then check if the vector contents are different.
      for (unsigned int i = 0; i < l.acquired_locations.size(); ++i) {
        if (l.acquired_locations[i] < r.acquired_locations[i]) return true;
        if (l.acquired_locations[i] > r.acquired_locations[i]) return false;
      }
      for (unsigned int i = 0; i < l.relieved_locations.size(); ++i) {
        if (l.relieved_locations[i] < r.relieved_locations[i]) return true;
        if (l.relieved_locations[i] > r.relieved_locations[i]) return false;
      }
      for (unsigned int i = 0; i < l.in_use_robots.size(); ++i) {
        if (l.in_use_robots[i] < r.in_use_robots[i]) return true;
        if (l.in_use_robots[i] > r.in_use_robots[i]) return false;
      }
      for (unsigned int i = 0; i < l.robots.size(); ++i) {
        if (l.robots[i] < r.robots[i]) return true;
        if (l.robots[i] > r.robots[i]) return false;
      }

      return false;
    }

    bool operator==(const State& l, const State& r ) {
      return l.graph_id == r.graph_id &&
        l.direction == r.direction &&
        l.precision == r.precision &&
        l.from_graph_node == r.from_graph_node &&
        l.robot_gave_direction == r.robot_gave_direction &&
        l.acquired_locations == r.acquired_locations && 
        l.relieved_locations == r.relieved_locations && 
        l.in_use_robots == r.in_use_robots &&
        l.robots == r.robots;
    }

    std::ostream& operator<<(std::ostream& stream, const State& s) {
      stream << "[" << s.graph_id << ", " 
        << s.direction << ", (";
      for (unsigned int i = 0; i < s.robots.size(); ++i) {
        stream << s.robots[i].graph_id << ":" << s.robots[i].destination;
        if (i != s.robots.size() - 1)
          stream << ","; 
      }
      stream << "), ";
      if (s.in_use_robots.size() == 0) {
        stream << "No robots in use";
      } else {
        for (unsigned int i = 0; i < s.in_use_robots.size(); ++i) {
          stream << s.in_use_robots[i].robot_id << ":" <<
            s.in_use_robots[i].destination << "->" <<
            s.in_use_robots[i].direction << ":" <<
            s.in_use_robots[i].reached_destination;
          if (i != s.in_use_robots.size() - 1)
            stream << ","; 
        }
      }
      stream << "]";
      return stream;
    }

  } /* mrn */

} /* bwi_guidance_solver */
