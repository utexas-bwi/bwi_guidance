#ifndef BASE_ROBOT_POSITIONER_IMH4RD8H
#define BASE_ROBOT_POSITIONER_IMH4RD8H

#include <boost/shared_ptr.hpp>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <topological_mapper/point_utils.h>
#include <topological_mapper/map_loader.h>
#include <topological_mapper/map_utils.h>
#include <topological_mapper/graph.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <ros/ros.h>

namespace bwi_exp1 {

  class BaseRobotPositioner {

    public:
      BaseRobotPositioner(boost::shared_ptr<ros::NodeHandle>& nh);
      virtual ~BaseRobotPositioner();

      virtual void startExperimentInstance(int instance_number) = 0;
      virtual void finalizeExperimentInstance(int instance_number) = 0;
      virtual void odometryCallback(const nav_msgs::Odometry::ConstPtr) = 0;

      void produceDirectedArrow(float orientation, cv::Mat& image);

      bool checkClosePoses(const geometry_msgs::Pose& p1,
          const geometry_msgs::Pose& p2);
      bool teleportEntity(const std::string& entity, 
          const geometry_msgs::Pose& pose);

      geometry_msgs::Pose positionRobot(
          const topological_mapper::Point2f& from,
          const topological_mapper::Point2f& at,
          const topological_mapper::Point2f& to);

    private:
      cv::Mat up_arrow_;
      boost::shared_ptr<ros::NodeHandle> nh_;

      bool gazebo_available_;
      ros::ServiceClient get_gazebo_model_client_;
      ros::ServiceClient set_gazebo_model_client_;

      bool debug_;
      double search_distance_;
      boost::shared_ptr<topological_mapper::MapLoader> mapper_;
      topological_mapper::Graph graph_;
      nav_msgs::OccupancyGrid map_;
      nav_msgs::MapMetaData map_info_;

  };

} /* bwi_exp1 */

#endif /* end of include guard: BASE_ROBOT_POSITIONER_IMH4RD8H */
