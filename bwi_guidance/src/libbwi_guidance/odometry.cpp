#include <bwi_guidance/odometry.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace bwi_guidance {
  
  bool readOdometry(const std::string& file, 
      std::vector<LocationStamped>& path) {

    std::ifstream fin(file.c_str());
    if (!fin) {
      path.clear();
      return false;
    }
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);

    float start_time = 0;

    path.clear();
    for (size_t i = 0; i < doc["odometry"].size(); ++i) {
      const YAML::Node& stamped_location = doc["odometry"][i];
      LocationStamped location;
      stamped_location[0] >> location.location.x;
      stamped_location[1] >> location.location.y;
      int stamp_idx = 2;
      if (stamped_location.size() == 4) {
        stamped_location[2] >> location.location.yaw;
        stamp_idx = 3;
      } else {
        location.location.yaw = 0;
      }
      if (i == 0) {
        stamped_location[stamp_idx] >> start_time;
      }
      float current_time;
      stamped_location[stamp_idx] >> current_time;
      location.seconds_since_start = current_time - start_time;
      path.push_back(location);
    }

    bool success;
    doc["success"] >> success;
    return success;
  }

} /* bwi_guidance */
