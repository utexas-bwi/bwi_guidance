/**
 * @file /include/clingo_interface_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef clingo_interface_gui_QNODE_HPP_
#define clingo_interface_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <QThread>
#include <QStringListModel>

#include <topological_mapper/map_loader.h>
#include <tf/transform_listener.h>
#include <clingo_interface_gui/ClingoInterfaceAction.h>
#include <clingo_interface/door_handler.h>
#include <clingo_interface/gazebo_handler.h>
#include <clingo_interface/cost_estimator.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace clingo_interface_gui {

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
      void clingoInterfaceHandler(
          const clingo_interface_gui::ClingoInterfaceGoalConstPtr &req);

      /* Get robot location */
      void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom);

      bool newLocationReceived(const std::string& loc);

      /* Display stuff */
      std::string display_text_;
      bool button1_enabled_;
      std::string button1_text_;
      bool button2_enabled_;
      std::string button2_text_;
      cv::Mat generated_image_;
      bool location_box_enabled_;

    Q_SIGNALS:
      void rosShutdown();
      void updateFrameInfo();

    private:
      int init_argc;
      char** init_argv;

      void senseDoorProximity(std::vector<clingo_interface_gui::ClingoFluent>& fluents, int door_idx = -1);
      bool executeRobotGoal(const geometry_msgs::PoseStamped& pose);

      /* Ros Stuff */
      boost::shared_ptr<ros::NodeHandle> nh_;
      boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > robot_controller_;
      ros::Subscriber odom_subscriber_;
      boost::shared_ptr<actionlib::SimpleActionServer<
          clingo_interface_gui::ClingoInterfaceAction> > as_;

      /* Robot Location */
      float robot_x_;
      float robot_y_;
      float robot_yaw_;

      /* Feedback from gui */
      bool location_received_;
      std::string person_name_;
      std::string person_location_;

      /* DoorHandler */
      std::string map_file_;
      std::string door_file_;
      std::string location_file_;
      boost::shared_ptr<tf::TransformListener> tf_;
      boost::shared_ptr<topological_mapper::MapLoader> mapper_;
      boost::shared_ptr<clingo_interface::DoorHandler> handler_;
      boost::shared_ptr<clingo_interface::GazeboHandler> gh_;
      boost::shared_ptr<clingo_interface::CostEstimator> ce_;

      bool auto_door_open_enabled_;
      int close_door_idx_;
      int prev_door_idx_;
      
  };

}  // namespace clingo_interface_gui

#endif /* clingo_interface_gui_QNODE_HPP_ */
