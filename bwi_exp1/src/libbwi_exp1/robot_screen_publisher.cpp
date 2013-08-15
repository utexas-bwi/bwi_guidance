#include <bwi_exp1/robot_screen_publisher.h>
#include <boost/range/adaptor/map.hpp>
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace bwi_exp1 {

  RobotScreenPublisher::RobotScreenPublisher(
      boost::shared_ptr<ros::NodeHandle>& nh) : nh_(nh) {}

  RobotScreenPublisher::~RobotScreenPublisher() {
    if (publishing_thread_) {
      publishing_thread_->join();
    }
    ROS_INFO_STREAM("RobotScreenPublisher shutting down...");
  }

  void RobotScreenPublisher::addRobot(const std::string& robot_id) {
    if (robot_image_publisher_.find(robot_id) != robot_image_publisher_.end()) {
      ROS_ERROR_STREAM("RobotScreenPublisher: Id " << robot_id << " exists!");
      return;
    }
    robot_image_publisher_[robot_id] = 
      nh_->advertise<sensor_msgs::Image>(robot_id + "/image", 1);
    robot_image_[robot_id] = cv::Mat::zeros(120, 160, CV_8UC3);
    robot_image_lock_[robot_id].reset(new boost::mutex);
  }

  void RobotScreenPublisher::updateImage(const std::string& robot_id, 
      const cv::Mat& mat) {
    boost::mutex::scoped_lock lock(*(robot_image_lock_[robot_id]));
    robot_image_[robot_id] = mat;
  }

  void RobotScreenPublisher::start() {
    ROS_INFO_STREAM("RobotScreenPublisher starting up...");
    publishing_thread_.reset(
        new boost::thread(boost::bind(&RobotScreenPublisher::run, this)));
  }

  void RobotScreenPublisher::run() {
    ros::Rate rate(5);
    while (ros::ok()) {
      BOOST_FOREACH(const std::string& robot_id, 
          robot_image_publisher_ | boost::adaptors::map_keys) {
        boost::mutex::scoped_lock lock(*(robot_image_lock_[robot_id]));
        cv_bridge::CvImage out_image;
        out_image.header.frame_id = robot_id + "/laptop_screen_link";
        out_image.header.stamp = ros::Time::now();
        out_image.encoding = sensor_msgs::image_encodings::BGR8;
        out_image.image = robot_image_[robot_id];
        robot_image_publisher_[robot_id].publish(out_image.toImageMsg());
      }
      rate.sleep();
    }
  }
  
} /* bwi_exp1 */
