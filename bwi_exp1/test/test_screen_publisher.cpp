#include <bwi_exp1/robot_screen_publisher.h>
#include <ros/package.h>
#include <opencv/highgui.h>

using namespace bwi_exp1;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "test_robot_positioner");
  boost::shared_ptr<ros::NodeHandle> nh;
  nh.reset(new ros::NodeHandle());
  RobotScreenPublisher rp(nh);
  rp.addRobot("test_robot1");
  ros::Rate r(1);
  cv::Mat img[2];
  img[0] = cv::imread(ros::package::getPath("bwi_exp1") + "/images/Up.png");
  img[1] = cv::imread(ros::package::getPath("bwi_exp1") + "/images/Down.png");
  int counter = 0;
  rp.start();
  while (ros::ok()) {
    rp.updateImage("test_robot1", img[counter % 2]);
    ++counter;
    r.sleep();
  }
  return 0;
}

