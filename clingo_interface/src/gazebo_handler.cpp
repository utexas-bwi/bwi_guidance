#include <ros/ros.h>
#include <clingo_interface/gazebo_handler.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "clingo_gazebo_handler");
  clingo_interface::GazeboHandler gh;
  ros::spin();
  return 0;
}
