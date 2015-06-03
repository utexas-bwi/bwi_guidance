#include <bwi_guidance_solver/mrn/base_robot_navigator.h>
#include <yaml-cpp/yaml.h>

using namespace bwi_guidance_solver::mrn;

bool waiting_for_robots = true;
boost::shared_ptr<TaskGenerationModel> model;
std::vector<std::string> available_robot_list;
std::vector<std::vector<int> > task_list;

std::map<std::string, std::vector<int> > robot_tasks_from_yaml;

void readRobotTasksFromFile(std::string &filename) {

  ROS_WARN_STREAM("Reading file: " << filename);
  std::ifstream fin(filename.c_str());
  YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
  doc = YAML::Load(fin);
#else
  YAML::Parser parser(fin);
  parser.GetNextDocument(doc);
#endif
  for (size_t i = 0; i < doc.size(); ++i) {
    std::string key;
    doc[i]["name"] >> key;
    ROS_WARN_STREAM("Found robot " << key);
    std::vector<int> tasks;
    const YAML::Node& tasks_node = doc[i]["tasks"];
    for (size_t j = 0; j < tasks_node.size(); ++j) {
      int task;
      tasks_node[j] >> task;
      tasks.push_back(task);
    }
    robot_tasks_from_yaml[key] = tasks;
  }
  fin.close();

}

class SimpleTaskGenerationModel : public TaskGenerationModel {

  public:

    virtual ~SimpleTaskGenerationModel() {}

    virtual void generateNewTaskForRobot(int robot_id, RobotState &robot, RNG &rng) {
      for (int i = 0; i < task_list[robot_id].size(); ++i) {
        if (robot.tau_d == -1) {
          robot.tau_d = task_list[robot_id][0];
          robot.tau_t = 0.0f;
          robot.tau_total_task_time = 30.0f;
          robot.tau_u = 0.5f;
          break;
        } else if (task_list[robot_id][i] == robot.tau_d) {
          robot.tau_d = task_list[robot_id][(i+1)%task_list[robot_id].size()];
          robot.tau_t = 0.0f;
          robot.tau_total_task_time = 30.0f;
          robot.tau_u = 0.5f;
          break;
        }
      }
    }
};

bool start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  waiting_for_robots = false;
  return true;
}

void availableRobotHandler(const bwi_msgs::AvailableRobotArray::ConstPtr available_robots) {
  BOOST_FOREACH(const bwi_msgs::AvailableRobot &robot, available_robots->robots) {
    if (std::find(available_robot_list.begin(), available_robot_list.end(), robot.name) == available_robot_list.end()) {
      available_robot_list.push_back(robot.name);
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "base_robot_navigator");

  boost::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle);

  ros::ServiceServer start_server = nh->advertiseService("start_mrn", &start);
  ros::Subscriber available_robots_subscriber = nh->subscribe("/available_robots", 1, availableRobotHandler);

  while(waiting_for_robots) {
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  }

  ros::NodeHandle private_nh("~");
  std::string tasks_file;
  if (!private_nh.getParam("tasks_file", tasks_file)) {
    ROS_FATAL("Tasks file parameter ~tasks_file not specified!");
    return -1;
  }
  readRobotTasksFromFile(tasks_file);

  BOOST_FOREACH(const std::string &robot_name, available_robot_list) {
    if (robot_tasks_from_yaml.find(robot_name) != robot_tasks_from_yaml.end()) {
      task_list.push_back(robot_tasks_from_yaml[robot_name]);
    } else {
      ROS_FATAL_STREAM("Robot " + robot_name + " is available, but no tasks provided for this robot!");
      return -1;
    }
  }

  model.reset(new SimpleTaskGenerationModel());
  BaseRobotNavigator brn(nh, available_robot_list, model);
  brn.start();

  return 0;
}
