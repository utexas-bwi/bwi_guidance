#ifndef STRUCTURES_NQP4EY6
#define STRUCTURES_NQP4EY6

#include <topological_mapper/structures/point.h>
#include <fstream>
#include <string>
#include <yaml-cpp/yaml.h>

namespace clingo_interface {

  class Location {
    public:
      std::string name;
      topological_mapper::Point2f loc;
  };

  inline void readLocationFile(const std::string& filename, 
      std::vector<Location>& locations) {
    std::ifstream fin(filename.c_str());
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);

    locations.clear();
    for (size_t i = 0; i < doc.size(); i++) {
      Location location;
      const YAML::Node &loc_node = doc[i]["loc"];
      for (size_t j = 0; j < 2; ++j) {
        loc_node[0] >> location.loc.x;
        loc_node[1] >> location.loc.y;
      }
      doc[i]["name"] >> location.name;
      locations.push_back(location);
    }

  }

  class Door {
    public:
      std::string name;
      topological_mapper::Point2f approach_points[2];
      float approach_yaw[2];
      topological_mapper::Point2f corners[4];
  };

  inline void readDoorFile(const std::string& filename, std::vector<Door>& doors) {
    std::ifstream fin(filename.c_str());
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);

    doors.clear();
    for (size_t i = 0; i < doc.size(); i++) {
      Door door;
      const YAML::Node &door_node = doc[i]["corners"];
      for (size_t j = 0; j < 4; ++j) {
        door_node[j][0] >> door.corners[j].x;
        door_node[j][1] >> door.corners[j].y;
      }
      const YAML::Node &approach_node = doc[i]["approach"];
      for (size_t j = 0; j < 2; ++j) {
        approach_node[j][0] >> door.approach_points[j].x; 
        approach_node[j][1] >> door.approach_points[j].y; 
        approach_node[j][2] >> door.approach_yaw[j]; 
      }
      doc[i]["name"] >> door.name;
      doors.push_back(door);
    }
  }

} /* namespace clingo_interface */

#endif /* end of include guard: STRUCTURES_NQP4EY6 */
