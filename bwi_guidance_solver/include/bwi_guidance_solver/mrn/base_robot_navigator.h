#ifndef BWI_GUIDANCE_BASE_ROBOT_NAVIGATOR_H
#define BWI_GUIDANCE_BASE_ROBOT_NAVIGATOR_H

#include <actionlib/client/simple_action_client.h>
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
#include <std_srvs/Empty.h>

#include <bwi_msgs/AvailableRobotArray.h>
#include <bwi_guidance_msgs/ExperimentStatus.h>
#include <bwi_guidance_msgs/MultiRobotNavigationAction.h>
#include <bwi_guidance_solver/mrn/transition_model.h>
#include <bwi_guidance_solver/mrn/extended_structures.h>

namespace bwi_guidance_solver {

  namespace mrn {

    class BaseRobotNavigator {

      public:

        BaseRobotNavigator(const boost::shared_ptr<ros::NodeHandle>& nh,
                           const boost::shared_ptr<TaskGenerationModel>& model);
        virtual ~BaseRobotNavigator();

        bool human_location_available_;
        ros::Subscriber human_location_subscriber_;
        void humanLocationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr human_pose);
        geometry_msgs::Pose human_location_;

        // std_srvs/Empty call that freezes the available robot list and starts patrolling them via the concurrent thread.
        // Doesn't actually provide the goal.
        ros::ServiceServer start_server_;
        void startMultiRobotNavigator(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        // This needs a mutex inside so that the list of available robots is not changed once the action server receives
        // a request.
        // TODO: Atleast assume that a robot that has become available won't suddenly vanish, for now.
        ros::Subscriber available_robots_subscriber_;
        void availableRobotHandler(const bwi_msgs::AvailableRobotArray::ConstPtr available_robots);
        std::vector<std::string> available_robot_list_;

        // While available robot list is frozen, we can't add more robots.
        bool multi_robot_navigator_started_;

        std::vector<boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > > robot_controller_; /* Directly corresponds to vector id in current state. */

        std::vector<bool> robot_location_available_;
        std::vector<boost::shared_ptr<ros::Subscriber> > robot_location_subscriber_;
        void robotLocationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr robot_pose, int robot_idx);
        std::vector<geometry_msgs::Pose> robot_location_;

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
        void getBestAction();

        void getNextTaskForRobot(int robot_id, RobotState &rs);

        // This is a step of time during WAIT where UCT can do its things. The timeout is based on the frequency of the
        // controller thread minus the processing time required by that thread.
        // Do nothing in this function by default.
        virtual void compute(float timeout);

        // TODO: Move to common
        void produceDirectedArrow(float orientation, cv::Mat& image);
        void produceDirectedArrowAlt(float orientation, cv::Mat& image);

      protected:

        ExtendedState system_state;

        boost::shared_ptr<ros::NodeHandle> nh_;
        boost::shared_ptr<actionlib::SimpleActionServer<bwi_guidance_msgs::MultiRobotNavigationAction> > as_;
        bool episode_in_progress_;

        int goal_node_id_;

        boost::shared_ptr<boost::thread> controller_thread_;
        void runControllerThread();
        float controller_thread_frequency_;

        boost::shared_ptr<bwi_mapper::MapLoader> mapper_;
        bwi_mapper::Graph graph_;
        nav_msgs::OccupancyGrid map_;
        nav_msgs::MapMetaData map_info_;

        boost::shared_ptr<TaskGenerationModel> task_generation_model_;

    };

  } /* mrn */

} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_BASE_ROBOT_NAVIGATOR_H */
