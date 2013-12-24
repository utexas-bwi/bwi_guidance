#include <bwi_guidance/robots.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace bwi_guidance {

  void operator >> (const YAML::Node& node, PathPoint& point) {
    node["id"] >> point.graph_id;
    node["robot"] >> point.robot_present;
  }

  void operator >> (const YAML::Node& node, Robot& robot) {
    node["id"] >> robot.id;
    node["default_loc"][0] >> robot.default_loc.x;
    node["default_loc"][1] >> robot.default_loc.y;
    robot.default_loc.yaw = 0;
  }

  void operator >> (const YAML::Node& node, DefaultRobots& er) {
    er.robots.clear();
    for (size_t i = 0; i < node["robots"].size(); ++i) {
      Robot robot;
      node["robots"][i] >> robot;
      er.robots.push_back(robot);
    }
  }

  void readDefaultRobotsFromFile(const std::string& file, 
      DefaultRobots& er) {

    std::ifstream fin(file.c_str());
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);
    doc >> er;
  }

  void operator >> (const YAML::Node& node, DCInstanceRobots& instance) {
    instance.path.clear();
    for (size_t i = 0; i < node["path"].size(); ++i) {
      PathPoint path_point;
      node["path"][i] >> path_point;
      instance.path.push_back(path_point);
    }
    instance.robots.clear();
    for (size_t i = 0; i < node["robots"].size(); ++i) {
      Location robot;
      node["robots"][i]["loc_x"] >> robot.x;
      node["robots"][i]["loc_y"] >> robot.y;
      node["robots"][i]["yaw"] >> robot.yaw;
      instance.robots.push_back(robot);
    }
  }

  void operator >> (const YAML::Node& node, 
      DCInstanceGroupRobots& instance_group) {
    node["prefix"] >> instance_group.prefix;
    instance_group.instances.clear();
    for (size_t i = 0; i < node["instances"].size(); ++i) {
      DCInstanceRobots instance;
      node["instances"][i] >> instance;
      instance_group.instances.push_back(instance);
    }
  }

  void operator >> (const YAML::Node& node, DCExperimentRobots& er) {
    er.instance_groups.clear();
    for (size_t i = 0; i < node["instance_groups"].size(); ++i) {
      DCInstanceGroupRobots group;
      node["instance_groups"][i] >> group;
      er.instance_groups.push_back(group);
    }
  }

  void readDCExperimentRobotsFromFile(const std::string& file, 
      DCExperimentRobots& er) {

    std::ifstream fin(file.c_str());
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);
    doc >> er;
  }

  const DCInstanceRobots& getDCInstance(const std::string& instance_name,
      const DCExperimentRobots& er) {

    BOOST_FOREACH(const DCInstanceGroupRobots& ig, er.instance_groups) {
      std::string prefix = ig.prefix;
      for (size_t i = 0; i < ig.instances.size(); ++i) {
        std::string compound_name = prefix + "_" + 
            boost::lexical_cast<std::string>(i);
        if (instance_name == compound_name)
          return ig.instances[i];
      }
    }

    std::cout << "The robot file does not contain an instance named " <<
        instance_name;
    exit(-1);
    return er.instance_groups[0].instances[0];
  }
  
} /* bwi_guidance */
