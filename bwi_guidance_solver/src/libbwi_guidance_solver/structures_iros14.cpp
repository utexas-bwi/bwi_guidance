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
    return (l.graph_id < r.graph_id) ||
      ((l.graph_id == r.graph_id) && (l.destination < r.destination)); 
  }
  
  bool operator==(const RobotStateIROS14& l, const RobotStateIROS14& r) {
    return (l.graph_id == r.graph_id) && (l.destination == r.destination); 
  }

  bool operator>(const RobotStateIROS14& l, const RobotStateIROS14& r) {
    return (l.graph_id > r.graph_id) ||
      ((l.graph_id == r.graph_id) && (l.destination > r.destination)); 
  }

  bool operator<(const StateIROS14& l, const StateIROS14& r ) {
    if (l.graph_id < r.graph_id) return true;
    if (l.graph_id > r.graph_id) return false;

    if (l.direction < r.direction) return true;
    if (l.direction > r.direction) return false;

    if (l.robot_destination < r.robot_destination) return true;
    if (l.robot_destination > r.robot_destination) return false;

    if (l.robot_direction < r.robot_direction) return true;
    if (l.robot_direction > r.robot_direction) return false;

    if (l.selected_robot < r.selected_robot) return true;
    if (l.selected_robot > r.selected_robot) return false;

    // Shouldn't ever be running with different robot sizes.
    // if (l.robots.size() < r.robots.size()) return true;
    // if (l.robots.size() > r.robots.size()) return false;

    for (unsigned int i = 0; i < l.robots.size(); ++i) {
      if (l.robots[i] < r.robots[i]) return true;
      if (l.robots[i] > r.robots[i]) return false;
    }

    return false;
  }

  bool operator==(const StateIROS14& l, const StateIROS14& r ) {
    bool retval = (l.graph_id == r.graph_id &&
        l.direction == r.direction &&
        l.robot_direction == r.robot_direction &&
        l.robot_destination == r.robot_destination &&
        l.selected_robot == r.selected_robot);

    if (!retval) return retval;

    for (unsigned int i = 0; i < l.robots.size(); ++i) {
      retval = retval && (l.robots[i] == r.robots[i]);
    }
    return retval;
  }

  std::ostream& operator<<(std::ostream& stream, const StateIROS14& s) {
    stream << "[" << s.graph_id << ", " 
        << s.direction << ", (";
    for (unsigned int i = 0; i < s.robots.size(); ++i) {
      stream << s.robots[i].graph_id << ":" << s.robots[i].destination;
      if (i != s.robots.size() - 1)
        stream << ","; 
    }
    if (s.selected_robot == NONE) {
      stream << "), No Assigned Robot]";
    } else {
      stream << "), " << s.selected_robot << ":" << s.robot_destination << "->"
        << s.robot_direction;
    }
    return stream;
  }

} /* bwi_guidance */
