#include <bwi_guidance_solver/mrn/base_robot_navigator.h>

using namespace bwi_guidance_solver::mrn;

bool waiting_for_robots = true;
boost::shared_ptr<TaskGenerationModel> model;
std::vector<std::string> available_robot_list;
std::vector<std::vector<int> > task_list;

class SimpleTaskGenerationModel : public TaskGenerationModel {

  public:

    virtual ~SimpleTaskGenerationModel() {}

    virtual void generateNewTaskForRobot(int robot_id, RobotState &robot, RNG &rng) {
      for (int i = 0; i < task_list[robot_id].size(); ++i) {
        if (robot.tau_d == -1) {
          robot.tau_d = task_list[robot_id][0];
          robot.tau_t = 0.0f;
          robot.tau_total_task_time = 30.0f;
          robot.tau_u = 0.0f;
          break;
        } else if (task_list[robot_id][i] == robot.tau_d) {
          robot.tau_d = task_list[robot_id][(i+1)%task_list[robot_id].size()];
          robot.tau_t = 0.0f;
          robot.tau_total_task_time = 30.0f;
          robot.tau_u = 0.0f;
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

  BOOST_FOREACH(const std::string &robot_name, available_robot_list) {
    std::vector<int> robot_task_list;
    if (robot_name == "marvin") {
      robot_task_list.push_back(42);
      robot_task_list.push_back(44);
      robot_task_list.push_back(43);
      robot_task_list.push_back(41);
      robot_task_list.push_back(39);
      robot_task_list.push_back(38);
      robot_task_list.push_back(37);
      robot_task_list.push_back(40);
    } else if (robot_name == "roberto") {
      robot_task_list.push_back(13);
      robot_task_list.push_back(47);
      robot_task_list.push_back(46);
      robot_task_list.push_back(45);
      robot_task_list.push_back(48);
    } else {
      robot_task_list.push_back(21);
      robot_task_list.push_back(26);
      robot_task_list.push_back(27);
      robot_task_list.push_back(28);
      robot_task_list.push_back(17);
    }
    task_list.push_back(robot_task_list);
  }

  model.reset(new SimpleTaskGenerationModel());
  BaseRobotNavigator brn(nh, available_robot_list, model);
  brn.start();

  return 0;
}
