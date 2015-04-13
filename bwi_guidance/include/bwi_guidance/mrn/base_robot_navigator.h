#ifndef BWI_GUIDANCE_BASE_ROBOT_NAVIGATOR_H
#define BWI_GUIDANCE_BASE_ROBOT_NAVIGATOR_H

#include <actionlib/server/simple_action_server.h>
#include <boost/shared_ptr.hpp>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <bwi_mapper/point_utils.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_mapper/map_utils.h>
#include <bwi_mapper/graph.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <ros/ros.h>

#include <bwi_guidance/experiment.h>
#include <bwi_guidance/robot_screen_publisher.h>
#include <bwi_guidance/robots.h>
#include <bwi_guidance_msgs/ExperimentStatus.h>
#include <bwi_guidance_msgs/MultiRobotNavigationAction.h>

namespace bwi_guidance {

  class BaseMultiRobotNavigator {

    // TODO add current_state. The only  and decision.
    public:

      BaseRobotNavigator(boost::shared_ptr<ros::NodeHandle>& nh);
      virtual ~BaseRobotNavigator();

      // TODO This is provided from somewhere, not necessarily gazebo.
      void humanPositionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr odom);

      // TODO We need to know the positions of all the robots to keep the state up to date
      void robotPositionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr robot);

      // While available robot list is frozen, we can't add more robots.
      bool freeze_available_robot_list_;

      // This needs a mutex inside so that the list of available robots is not changed once the action server receives
      // a request.
      // TODO: Atleast assume that a robot that has become available won't suddenly vanish, for now.
      void availableRobotSubscriber();
      std::vector<std::string> available_robot_list;
      std::vector</* robot controller */> robot_controller_; /* Directly corresponds to vector id in current state. */

      // TODO Keep a separate thread on a timer that computes the current state based on human and robot positions, and
      // keeps track on the amount of time a service robot has spent on a location and figure out when a new service
      // task is necessary. This thread should also be responsible for requesting the best action and executing said
      // action. Since this thread is responsible for sending navigation tasks to the robots, this thread should also
      // monitor each robot and figure out what the current status is.
      // This thread should also frontload all actions to get the cummulative effect until WAIT is called.
      // Probably run

      // Start here.
      // TODO Expose an action_server for requesting assistance that terminates once the human reaches the goal. This
      // should compute the initial state, and start the thread that attempts to achieve the task. Once the thread
      // finishes or a cancel request is received, stop the task!

      /* Once WAIT is returned, clean the MCTS state - DOWNSTREAM! */
      virtual void getBestAction() = 0;

      // This is a step of time during WAIT where UCT can do its things. The timeout is based on the frequency of the
      // controller thread minus the processing time required by that thread.
      // Do nothing in this function by default.
      virtual void compute(float timeout);

      // TODO: Move to common
      void produceDirectedArrow(float orientation, cv::Mat& image);
      void produceDirectedArrowAlt(float orientation, cv::Mat& image);

    protected:

      boost::shared_ptr<ros::NodeHandle> nh_;
      boost::shared_ptr<actionlib::SimpleActionServer<bwi_guidance_msgs::MultiRobotNavigationAction> > as_;
      bool cancel_controller_thread_;

      boost::shared_ptr<boost::thread> controller_thread_;
      void runControllerThread
      float controller_thread_frequency_;

      boost::shared_ptr</* TaskGenerationModel */>

      ros::Subscriber odometry_subscriber_;

      bool debug_;
      double search_distance_;
      boost::shared_ptr<bwi_mapper::MapLoader> mapper_;
      bwi_mapper::Graph graph_;
      nav_msgs::OccupancyGrid map_;
      nav_msgs::OccupancyGrid inflated_map_;
      nav_msgs::MapMetaData map_info_;

      RobotScreenPublisher robot_screen_publisher_;

      std::map<std::string, float> robot_screen_orientations_;
      std::map<std::string, bool> robot_ok_;
      boost::mutex robot_modification_mutex_;

      int current_instance_;
      bool instance_in_progress_;
      bool prev_msg_ready_;
      ros::Subscriber experiment_status_subscriber_;

  };

} /* bwi_guidance */

#endif /* end of include guard: BWI_GUIDANCE_BASE_ROBOT_NAVIGATOR_H */
