#include <bwi_guidance/users.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace bwi_guidance {

  void operator >> (const YAML::Node& node, User& user) {
    node["user"] >> user.id;
    node["name"] >> user.name;
    user.experiment_group_order.clear();
    user.experiment_group_size.clear();
    for (size_t i = 0; i < node["experiment_order"].size(); ++i) {
      std::string experiment_group_name;
      size_t experiment_group_size;
      node["experiment_order"][i]["name"] >> experiment_group_name;
      node["experiment_order"][i]["num"] >> experiment_group_size;
      user.experiment_group_order.push_back(experiment_group_name);
      user.experiment_group_size.push_back(experiment_group_size);
    }
  }

  void readUserDataFromFile(const std::string& file, std::vector<User>& users) {
    std::ifstream fin(file.c_str());
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);
    for (size_t i = 0; i < doc.size(); ++i) {
      User user;
      doc[i] >> user;
      users.push_back(user);
    }
  }

  std::vector<size_t> getAllUserIds(const std::vector<User>& users) {
    std::vector<size_t> user_ids;
    for (size_t i = 0; i < users.size(); ++i) {
      user_ids.push_back(i);
    }
    return user_ids;
  }

  std::vector<size_t> getUserIdsForOrdering(const std::vector<User>& users,
      std::vector<std::string> ordering) {
    std::vector<size_t> user_ids;
    for (size_t i = 0; i < users.size(); ++i) {
      if (std::equal(ordering.begin(), ordering.end(), 
            users[i].experiment_group_order.begin())) {
        user_ids.push_back(i);
      }
    }
    return user_ids;
  }
  

} /* bwi_guidance */
