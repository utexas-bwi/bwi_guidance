#include <bwi_guidance_solver/mrn/base_robot_navigator.h>

using namespace bwi_guidance_solver::mrn;

int main(int argc, char **argv) {
  ros::init(argc, argv, "base_robot_navigator");

  boost::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle);

  // TODO Construct the TaskGenerationModel substitute here.
  boost::shared_ptr<TaskGenerationModel> model;

  BaseRobotNavigator brn(nh, model);
  
  ros::spin();

  return 0;
}
