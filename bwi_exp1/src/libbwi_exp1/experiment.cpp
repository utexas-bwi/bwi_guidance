#include <bwi_exp1/experiment.h>
#include <fstream>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <boost/lexical_cast.hpp>

namespace bwi_exp1 {

  void operator >> (const YAML::Node& node, ExperimentPathPoint& point) {
    node["id"] >> point.graph_id;
    node["robot"] >> point.robot_present;
  }

  void operator >> (const YAML::Node& node, Experiment& exp) {
    node["start_x"] >> exp.start_loc.x;
    node["start_y"] >> exp.start_loc.y;
    node["start_yaw"] >> exp.start_loc.yaw;
    node["ball_x"] >> exp.ball_loc.x;
    node["ball_y"] >> exp.ball_loc.y;
    exp.path.clear();
    for (size_t i = 0; i < node["path"].size(); ++i) {
      ExperimentPathPoint path_point;
      node["path"][i] >> path_point;
      exp.path.push_back(path_point);
    }
  }

  void operator >> (const YAML::Node& node, ExperimentGroup& eg) {
    node["prefix"] >> eg.prefix;
    node["order"] >> eg.order;
    eg.experiments.clear();
    for (size_t i = 0; i < node["experiments"].size(); ++i) {
      Experiment exp;
      node["experiments"][i] >> exp;
      eg.experiments.push_back(exp);
    }
  }

  void operator >> (const YAML::Node& node, ExperimentRobot& robot) {
    node["id"] >> robot.id;
    node["default_loc"][0] >> robot.default_loc.x;
    node["default_loc"][1] >> robot.default_loc.y;
    robot.default_loc.yaw = 0;
  }

  void operator >> (const YAML::Node& node, ExperimentCollection& ec) {
    node["person_id"] >> ec.person_id;
    ec.robots.clear();
    for (size_t i = 0; i < node["robots"].size(); ++i) {
      ExperimentRobot robot;
      node["robots"][i] >> robot;
      ec.robots.push_back(robot);
    }
    ec.experiments.clear();
    for (size_t i = 0; i < node["experiments"].size(); ++i) {
      ExperimentGroup group;
      node["experiments"][i] >> group;
      ec.experiments.push_back(group);
    }
    node["ball_id"] >> ec.person_id;
  }

  void readExperimentCollectionFromFile(const std::string& file, 
      ExperimentCollection& ec) {

    std::ifstream fin(file.c_str());
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);
    doc >> ec;
  }

  void getExperimentNames(const ExperimentCollection& ec, 
      std::vector<std::string> &names) {
    names.clear();
    for (size_t i = 0; i < ec.experiments.size(); ++i) {
      const ExperimentGroup& eg = ec.experiments[i];
      for (size_t j = 0; j < eg.experiments.size(); ++j) {
        names.push_back(eg.prefix + "_" + 
            boost::lexical_cast<std::string>(j));
      }
    }
  }

  void computeOrderings(const ExperimentCollection& ec,
      std::vector< std::vector<std::string> >& orderings) {

    std::vector<std::string> base_order;
    base_order.resize(ec.experiments.size());
    std::vector<std::string> randomly_ordered_groups;
    for (size_t i = 0; i < ec.experiments.size(); ++i) {
      const ExperimentGroup& eg = ec.experiments[i];
      if (eg.order != -1) {
        base_order[eg.order] = eg.prefix;
      } else {
        randomly_ordered_groups.push_back(eg.prefix);
      }
    }

    // sort the randomly ordered groups into the lowest permutation
    std::sort(randomly_ordered_groups.begin(), randomly_ordered_groups.end());

    // Compute each permutation
    orderings.clear();
    do {
      std::vector<std::string> current_permutation;
      size_t base_order_size = 
        base_order.size() - randomly_ordered_groups.size();
      current_permutation.insert(current_permutation.end(), base_order.begin(),
          base_order.begin() + base_order_size);
      current_permutation.insert(current_permutation.end(),
          randomly_ordered_groups.begin(), randomly_ordered_groups.end());
      orderings.push_back(current_permutation);
    } while (std::next_permutation(randomly_ordered_groups.begin(), 
          randomly_ordered_groups.end()));

  }

} /* bwi_exp1 */
