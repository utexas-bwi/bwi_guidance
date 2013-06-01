#ifndef STRUCTURES_NQP4EY6
#define STRUCTURES_NQP4EY6

#include <topological_mapper/structures/point.h>
#include <fstream>
#include <string>
#include <yaml-cpp/yaml.h>

namespace clingo_interface {

  inline void readLocationFile(const std::string& filename, 
      std::vector<std::string>& locations, std::vector<int32_t> location_map) {
    std::ifstream fin(filename.c_str());
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);

    locations.clear();
    const YAML::Node &loc_node = doc["locations"];
    for (size_t i = 0; i < loc_node.size(); i++) {
      std::string location;
      loc_node[i] >> location;
      locations.push_back(location);
    }
    const YAML::Node &data_node = doc["data"];
    std::cout << data_node.size() << std::endl;
    location_map.resize(data_node.size());
    for (size_t i = 0; i < data_node.size(); i++) {
      data_node[i] >> location_map[i];
    }
    std::cout << "read location file" << std::endl;
  }

  class Door {
    public:
      std::string name;
      std::string approach_names[2];
      topological_mapper::Point2f approach_points[2];
      float approach_yaw[2];
      topological_mapper::Point2f corners[2];
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
      for (size_t j = 0; j < 2; ++j) {
        door_node[j][0] >> door.corners[j].x;
        door_node[j][1] >> door.corners[j].y;
      }
      const YAML::Node &approach_node = doc[i]["approach"];
      for (size_t j = 0; j < 2; ++j) {
        approach_node[j]["from"] >> door.approach_names[j];
        approach_node[j]["point"][0] >> door.approach_points[j].x; 
        approach_node[j]["point"][1] >> door.approach_points[j].y; 
        approach_node[j]["point"][2] >> door.approach_yaw[j]; 
      }
      doc[i]["name"] >> door.name;
      doors.push_back(door);
    }
    std::cout << "read door file" << std::endl;
  }

} /* namespace clingo_interface */

#endif /* end of include guard: STRUCTURES_NQP4EY6 */
