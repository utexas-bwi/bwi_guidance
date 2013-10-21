#include <ros/ros.h>
#include <clingo_interface/gazebo_handler.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "clingo_gazebo_handler");
  clingo_interface::GazeboHandler gh;
  ros::NodeHandle nh;
  int count = 0;
  ros::Rate r(1.0);
  while (ros::ok()) {
    ROS_INFO_STREAM("tick " << count);
    if (count == 10) gh.closeAllDoors();
    ros::spinOnce();
    ++count;
    r.sleep();
  }
  return 0;
}
