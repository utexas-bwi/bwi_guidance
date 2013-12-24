#ifndef ROBOT_SCREEN_PUBLISHER_CW5696X1
#define ROBOT_SCREEN_PUBLISHER_CW5696X1

#include <opencv/cv.h>
#include <ros/ros.h>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace bwi_guidance {

  class RobotScreenPublisher {

    public:

      RobotScreenPublisher(boost::shared_ptr<ros::NodeHandle>& nh);
      ~RobotScreenPublisher();

      void addRobot(const std::string& robot_id);
      void updateImage(const std::string& robot_id, const cv::Mat& image);
      void start();

    private:

      void run();

      boost::shared_ptr<boost::thread> publishing_thread_;
      boost::shared_ptr<ros::NodeHandle>& nh_;
      std::map<std::string, ros::Publisher> robot_image_publisher_;
      std::map<std::string, cv::Mat> robot_image_;
      std::map<std::string, boost::shared_ptr<boost::mutex> > robot_image_lock_;

  };
} /* bwi_guidance */

#endif /* end of include guard: ROBOT_SCREEN_PUBLISHER_CW5696X1 */
