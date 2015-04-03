#include <bwi_guidance/experiment.h>
#include <cstdlib>
#include <fstream>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <boost/lexical_cast.hpp>

namespace bwi_guidance {

  void operator >> (const YAML::Node& node, Instance& exp) {
    node["start_x"] >> exp.start_loc.x;
    node["start_y"] >> exp.start_loc.y;
    node["start_yaw"] >> exp.start_loc.yaw;
    node["ball_x"] >> exp.ball_loc.x;
    node["ball_y"] >> exp.ball_loc.y;
    node["max_duration"] >> exp.max_duration;
    node["max_robots"] >> exp.max_robots;
    node["is_tutorial"] >> exp.is_tutorial;
  }

  void operator >> (const YAML::Node& node, InstanceGroup& eg) {
    node["prefix"] >> eg.prefix;
    node["order"] >> eg.order;
    eg.experiments.clear();
    for (size_t i = 0; i < node["instances"].size(); ++i) {
      Instance exp;
      node["instances"][i] >> exp;
      eg.experiments.push_back(exp);
    }
  }

  void operator >> (const YAML::Node& node, Experiment& ec) {
    node["person_id"] >> ec.person_id;
    ec.experiments.clear();
    for (size_t i = 0; i < node["instance_groups"].size(); ++i) {
      InstanceGroup group;
      node["instance_groups"][i] >> group;
      ec.experiments.push_back(group);
    }
    node["ball_id"] >> ec.person_id;
  }

  void readExperimentFromFile(const std::string& file, 
      Experiment& ec) {

    std::ifstream fin(file.c_str());
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);
    doc >> ec;
  }

  void getInstanceNames(const Experiment& ec, 
      std::vector<std::string> &names) {
    names.clear();
    for (size_t i = 0; i < ec.experiments.size(); ++i) {
      const InstanceGroup& eg = ec.experiments[i];
      for (size_t j = 0; j < eg.experiments.size(); ++j) {
        names.push_back(eg.prefix + "_" + 
            boost::lexical_cast<std::string>(j));
      }
    }
  }

  void computeOrderings(const Experiment& ec,
      std::vector< std::vector<std::string> >& orderings) {

    std::vector<std::string> base_order;
    base_order.resize(ec.experiments.size());
    std::vector<std::string> randomly_ordered_groups;
    for (size_t i = 0; i < ec.experiments.size(); ++i) {
      const InstanceGroup& eg = ec.experiments[i];
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

  Instance& getInstance(Experiment& ec, size_t idx) {
    size_t count = 0;
    for (size_t i = 0; i < ec.experiments.size(); ++i) {
      InstanceGroup& eg = ec.experiments[i];
      for (size_t j = 0; j < eg.experiments.size(); ++j) {
        if (count == idx) {
          return eg.experiments[j];
        }
        count++;
      }
    }
    // Should not get here
    std::cout << "FATAL: The experiment does not contain an instance "
              << "with id: " << idx;
    exit(-1);
    return ec.experiments[0].experiments[0];
  }

  Instance& getInstance(Experiment& ec, std::string instance_name) {
    std::vector<std::string> names;
    getInstanceNames(ec, names);
    for (size_t i = 0; i < names.size(); ++i) {
      if (names[i] == instance_name) {
        return getInstance(ec, i);
      }
    }

    // Should not get here
    std::cout << "FATAL: The experiment does not contain an instance "
              << "with name: " << instance_name;
    exit(-1);
    return ec.experiments[0].experiments[0];
  }

} /* bwi_guidance */
