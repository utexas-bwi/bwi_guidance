#ifndef BASE_ROBOT_POSITIONER_IMH4RD8H
#define BASE_ROBOT_POSITIONER_IMH4RD8H

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

namespace bwi_guidance {

  namespace irm {

    class BaseRobotPositioner {

      public:
        BaseRobotPositioner(boost::shared_ptr<ros::NodeHandle>& nh);
        virtual ~BaseRobotPositioner();

        virtual void startExperimentInstance(
            const std::string& instance_name) = 0;
        virtual void odometryCallback(
            const nav_msgs::Odometry::ConstPtr odom) = 0;

        virtual void finalizeExperimentInstance();
        void produceDirectedArrow(float orientation, cv::Mat& image);
        void produceDirectedArrowAlt(float orientation, cv::Mat& image);

        void experimentCallback(const bwi_guidance_msgs::ExperimentStatus::ConstPtr es);
        geometry_msgs::Pose convert2dToPose(float x, float y, float yaw);
        bool checkClosePoses(const geometry_msgs::Pose& p1,
            const geometry_msgs::Pose& p2);
        bool teleportEntity(const std::string& entity,
            const geometry_msgs::Pose& pose);

        geometry_msgs::Pose positionRobot(
            const bwi_mapper::Point2f& from,
            const bwi_mapper::Point2f& at,
            const bwi_mapper::Point2f& to);

        void start();
        void run();

      protected:

        boost::shared_ptr<ros::NodeHandle> nh_;
        boost::shared_ptr<boost::thread> publishing_thread_;

        bool gazebo_available_;
        ros::Subscriber odometry_subscriber_;
        ros::Publisher position_publisher_;
        ros::ServiceClient get_gazebo_model_client_;
        ros::ServiceClient set_gazebo_model_client_;

        bool debug_;
        double search_distance_;
        boost::shared_ptr<bwi_mapper::MapLoader> mapper_;
        bwi_mapper::Graph graph_;
        nav_msgs::OccupancyGrid map_;
        nav_msgs::OccupancyGrid inflated_map_;
        nav_msgs::MapMetaData map_info_;

        RobotScreenPublisher robot_screen_publisher_;
        cv::Mat blank_image_;
        cv::Mat up_arrow_;
        cv::Mat u_turn_image_;
        DefaultRobots default_robots_;
        Experiment experiment_;

        std::map<std::string, geometry_msgs::Pose> robot_locations_;
        std::map<std::string, geometry_msgs::Pose> assigned_robot_locations_;
        std::map<std::string, float> robot_screen_orientations_;
        std::map<std::string, float> prev_orientation_;
        std::map<std::string, bool> robot_ok_;
        boost::mutex robot_modification_mutex_;

        int current_instance_;
        bool instance_in_progress_;
        bool prev_msg_ready_;
        ros::Subscriber experiment_status_subscriber_;

    };

  } /* namespace irm */
} /* bwi_guidance */

#endif /* end of include guard: BASE_ROBOT_POSITIONER_IMH4RD8H */
