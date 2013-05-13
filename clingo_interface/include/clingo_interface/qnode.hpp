/**
 * @file /include/clingo_interface/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef clingo_interface_QNODE_HPP_
#define clingo_interface_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <QThread>
#include <QStringListModel>

#include <topological_mapper/map_loader.h>
#include <tf/transform_listener.h>
#include <clingo_helpers/ClingoInterface.h>
#include <clingo_helpers/door_handler.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace clingo_interface {

/*****************************************************************************
** Class
*****************************************************************************/

  class QNode : public QThread {
    Q_OBJECT
    public:
      QNode(int argc, char** argv );
      virtual ~QNode();
      bool init();
      void run();

      /* Service callback */
      bool clingoInterfaceHandler(
          clingo_helpers::ClingoInterface::Request &req,
          clingo_helpers::ClingoInterface::Response &resp);

      /* Get robot location */
      void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom);

      /* Display stuff */
      std::string display_text_;
      bool button1_enabled_;
      std::string button1_text_;
      bool button2_enabled_;
      std::string button2_text_;
      cv::Mat generated_image_;

    Q_SIGNALS:
      void rosShutdown();
      void updateFrameInfo();

    private:
      int init_argc;
      char** init_argv;

      /* Command Publisher */
      ros::Publisher robot_controller_;
      ros::ServiceServer service_;
      ros::Subscriber odom_subscriber_;

      /* Robot Location */
      float robot_x_;
      float robot_y_;
      float robot_yaw_;

      /* DoorHandler */
      std::string map_file_;
      std::string door_file_;
      std::string location_file_;
      boost::shared_ptr<tf::TransformListener> tf_;
      boost::shared_ptr<topological_mapper::MapLoader> mapper_;
      boost::shared_ptr<clingo_helpers::DoorHandler> handler_;

  };

}  // namespace clingo_interface

#endif /* clingo_interface_QNODE_HPP_ */
