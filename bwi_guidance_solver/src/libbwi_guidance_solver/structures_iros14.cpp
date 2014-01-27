#include <bwi_guidance_solver/structures_iros14.h>

namespace bwi_guidance {


  ActionIROS14::ActionIROS14() : type(DO_NOTHING), at_graph_id(0), 
    guide_graph_id(0) {}
  ActionIROS14::ActionIROS14(ActionTypeIROS14 a, int dest, int dir) : type(a),
    at_graph_id(dest), guide_graph_id(dir) {}

  bool operator==(const ActionIROS14& l, const ActionIROS14& r) {
    return (l.type == r.type) && (l.at_graph_id == r.at_graph_id) &&
      (l.guide_graph_id == r.guide_graph_id);
  }

  bool operator<(const ActionIROS14& l, const ActionIROS14& r) {
    return (l.type < r.type) ||
      ((l.type == r.type) && (l.at_graph_id < r.at_graph_id)) ||
      ((l.type == r.type) && (l.at_graph_id == r.at_graph_id) && 
       (l.guide_graph_id < r.guide_graph_id)); 
  }

  const std::string ACTION_IROS14_NAMES[] = {
    "DO_NOTHING",
    "ASSIGN_ROBOT",
    "DIRECT_PERSON",
    "RELEASE_ROBOT"
  };
  std::ostream& operator<<(std::ostream& stream, const ActionIROS14& a) {

    stream << "[" << ACTION_IROS14_NAMES[a.type];
    if (a.type != DO_NOTHING)
      stream << " " << a.at_graph_id << "->" << a.guide_graph_id;
    stream << "]";

    return stream;
  }

  bool operator<(const RobotStateIROS14& l, const RobotStateIROS14& r) {
    return (l.graph_id < r.graph_id);
    // ||
    //   ((l.graph_id == r.graph_id) && (l.destination < r.destination)); 
  }
  
  bool operator==(const RobotStateIROS14& l, const RobotStateIROS14& r) {
    return (l.graph_id == r.graph_id);
    //&& (l.destination == r.destination); 
  }

  bool operator>(const RobotStateIROS14& l, const RobotStateIROS14& r) {
    return (l.graph_id > r.graph_id);
    // ||
    //   ((l.graph_id == r.graph_id) && (l.destination > r.destination)); 
  }

  bool operator<(const InUseRobotStateIROS14& l, const InUseRobotStateIROS14& r) {
    return (l.robot_id < r.robot_id) ||
      ((l.robot_id == r.robot_id) && (l.destination < r.destination)) ||
      ((l.robot_id == r.robot_id) && (l.destination == r.destination) &&
       (l.direction < r.direction)) ||
      ((l.robot_id == r.robot_id) && (l.destination == r.destination) &&
       (l.direction == r.direction) && 
       (l.reached_destination < r.reached_destination));
  }
  
  bool operator==(const InUseRobotStateIROS14& l, const InUseRobotStateIROS14& r) {
    return (l.robot_id == r.robot_id) && (l.destination == r.destination) &&
      (l.direction == r.direction) && 
      (l.reached_destination == r.reached_destination); 
  }

  bool operator>(const InUseRobotStateIROS14& l, const InUseRobotStateIROS14& r) {
    return (l.robot_id > r.robot_id) ||
      ((l.robot_id == r.robot_id) && (l.destination > r.destination)) ||
      ((l.robot_id == r.robot_id) && (l.destination == r.destination) &&
       (l.direction > r.direction)) ||
      ((l.robot_id == r.robot_id) && (l.destination == r.destination) &&
       (l.direction == r.direction) && 
       (l.reached_destination > r.reached_destination));
  }

  bool operator<(const StateIROS14& l, const StateIROS14& r ) {
    if (l.graph_id < r.graph_id) return true;
    if (l.graph_id > r.graph_id) return false;

    if (l.direction < r.direction) return true;
    if (l.direction > r.direction) return false;

    if (l.in_use_robots.size() < r.in_use_robots.size()) return true;
    if (l.in_use_robots.size() > r.in_use_robots.size()) return false;

    // int v = memcmp(&(l.in_use_robots[0]), &(r.in_use_robots[0]), 
    //       sizeof(InUseRobotStateIROS14) * l.in_use_robots.size());
    // if (v < 0) return true;
    // if (v > 0) return false;
    for (unsigned int i = 0; i < l.in_use_robots.size(); ++i) {
      if (l.in_use_robots[i] < r.in_use_robots[i]) return true;
      if (l.in_use_robots[i] > r.in_use_robots[i]) return false;
    }

    // Shouldn't ever be running with different robot sizes.
    assert(l.robots.size() == r.robots.size());
    // if (l.robots.size() < r.robots.size()) return true;
    // if (l.robots.size() > r.robots.size()) return false;

    // v = memcmp(&(l.robots[0]), &(r.robots[0]), 
    //       sizeof(RobotStateIROS14) * l.robots.size());
    // if (v < 0) return true;
    // if (v > 0) return false;
    for (unsigned int i = 0; i < l.robots.size(); ++i) {
      if (l.robots[i] < r.robots[i]) return true;
      if (l.robots[i] > r.robots[i]) return false;
    }

    return false;
  }

  bool operator==(const StateIROS14& l, const StateIROS14& r ) {
    return l.graph_id == r.graph_id &&
      l.direction == r.direction &&
      // memcmp(&(l.in_use_robots[0]), &(r.in_use_robots[0]), 
      //     sizeof(InUseRobotStateIROS14) * l.in_use_robots.size()) == 0 &&
      // memcmp(&(l.robots[0]), &(r.robots[0]), 
      //     sizeof(RobotStateIROS14) * l.robots.size()) == 0;
      l.in_use_robots == r.in_use_robots &&
      l.robots == r.robots;
    
    // bool retval = (l.graph_id == r.graph_id &&
    //     l.direction == r.direction &&
    //     l.in_use_robots.size() != r.in_use_robots.size());

    // for (unsigned int i = 0; retval && (i < l.in_use_robots.size()); ++i) {
    //   retval = retval && (l.in_use_robots[i] == r.in_use_robots[i]);
    // }

    // assert(l.robots.size() == r.robots.size());

    // for (unsigned int i = 0; retval && (i < l.robots.size()); ++i) {
    //   retval = retval && (l.robots[i] == r.robots[i]);
    // }

    // return retval;
  }

  std::ostream& operator<<(std::ostream& stream, const StateIROS14& s) {
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

} /* bwi_guidance */
