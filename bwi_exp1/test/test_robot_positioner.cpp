#include <bwi_exp1/base_robot_positioner.h>
#include <opencv/highgui.h>
#include <ros/ros.h>

using namespace bwi_exp1;

void testDirectedArrows(BaseRobotPositioner& rp) {
  cv::Mat image;
  rp.produceDirectedArrow(M_PI/3, image);
  cv::imshow("Output", image);
  cvWaitKey(0);
}

int main(int argc, char *argv[]) {
  
  ros::init(argc, argv, "test_robot_positioner");
  boost::shared_ptr<ros::NodeHandle> nh;
  nh.reset(new ros::NodeHandle());
  BaseRobotPositioner rp(nh);
  testDirectedArrows(rp);
  return 0;
}
