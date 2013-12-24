#include <bwi_guidance/base_robot_positioner.h>
#include <opencv/highgui.h>
#include <ros/ros.h>

using namespace bwi_guidance;

class TestRobotPositioner : public BaseRobotPositioner {
  public:
    TestRobotPositioner(boost::shared_ptr<ros::NodeHandle>& nh) :
        BaseRobotPositioner(nh) {}
    virtual ~TestRobotPositioner() {}
    virtual void startExperimentInstance(
        const std::string& instance_name) {}
    virtual void finalizeExperimentInstance(
        const std::string& instance_name) {}
    virtual void odometryCallback(const nav_msgs::Odometry::ConstPtr) {}
};

void testDirectedArrows(TestRobotPositioner& rp) {
  cv::Mat image;
  rp.produceDirectedArrow(M_PI/3, image);
  cv::imshow("Output", image);
  cvWaitKey(0);
}

int main(int argc, char *argv[]) {
  
  ros::init(argc, argv, "test_robot_positioner");
  boost::shared_ptr<ros::NodeHandle> nh;
  nh.reset(new ros::NodeHandle());
  TestRobotPositioner rp(nh);
  testDirectedArrows(rp);
  return 0;
}
