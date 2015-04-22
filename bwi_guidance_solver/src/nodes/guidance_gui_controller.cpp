#include <bwi_msgs/QuestionDialog.h>
#include <ros/ros.h>

#include <bwi_/Empty.h>
#include <bwi_guidance_msgs/MultiRobotNavigationAction.h>

bool episode_start_enabled_ = false;

std::vector<std::string> goal_names;
std::vector<int> goal_graph_ids;

void monitorEpisodeStartThread() {
  bwi_msgs::QuestionDialog srv;
  srv.request.type = bwi_msgs::QuestionDialog::QUESTION;
  srv.request.message = "Where would you like to go?";
  BOOST_FOREACH(const std::string& name, goal_names) {
    srv.request.options.push_back(name);
  }
  srv.request.timeout = 0.0f; // No timeout;
  enable_episode_start_service.call(srv);
  if (srv.response.index >= 0) {
    // TODO make the call to the action server.
  } else {
    // Do nothing. The request probably got preempted.
  }
}

void displayMessage() {

}

bool enableEpisodeStart(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  if (episode_start_enabled_) {
    ROS_WARN_NAMED("guidance_gui_controller", "Episode start already enabled!");
  }

  return true;
}

int main(int argc, const char *argv[]) {
  ros::init(argc, argv, "guidance_gui_controller");
  ros::NodeHandle nh;
  
  ros::ServiceServer enable_episode_start_service = nh.advertiseService("update_gui", enableEpisodeStart);

  return 0;
}
