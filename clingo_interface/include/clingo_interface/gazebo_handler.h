#ifndef GAZEBO_HANDLER_E8TY9EDI
#define GAZEBO_HANDLER_E8TY9EDI

#include <ros/ros.h>
#include <stdexcept>

#include <boost/algorithm/string/join.hpp>
#include <boost/regex.hpp>
#include <boost/shared_ptr.hpp>

#include <clingo_interface/structures.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Point32.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <topological_mapper/map_loader.h>
#include <topological_mapper/map_utils.h>
#include <topological_mapper/point_utils.h>

namespace clingo_interface {

  class GazeboHandler {

    public:

      GazeboHandler () {

        ros::NodeHandle nh, private_nh("~");

        std::vector<std::string> unavailable_parameters;
        std::string map_file, door_file, location_file;
        if (!(private_nh.getParam("map_file", map_file))) {
          unavailable_parameters.push_back("map_file");
        }
        if (!(private_nh.getParam("door_file", door_file))) {
          unavailable_parameters.push_back("door_file");
        }
        if (!(private_nh.getParam("location_file", location_file))) {
          unavailable_parameters.push_back("location_file");
        }
        if (!(private_nh.getParam("obstacle_urdf", obstacle_urdf_))) {
          unavailable_parameters.push_back("obstacle_urdf");
        }
        if (!(private_nh.getParam("door_urdf", door_urdf_))) {
          unavailable_parameters.push_back("door_urdf");
        }

        if (unavailable_parameters.size() != 0) {
          std::string message = "Following neccessary params not available: " +
            boost::algorithm::join(unavailable_parameters, ", ");
          ROS_INFO_STREAM(message);
          throw std::runtime_error(message);
        }

        readDoorFile(door_file, doors_);
        readLocationFile(location_file, locations_, location_map_);
        mapper_.reset(new topological_mapper::MapLoader(map_file));
        nav_msgs::OccupancyGrid grid;
        mapper_->getMap(grid);
        info_ = grid.info;

        get_gazebo_model_client_ =
          nh.serviceClient<gazebo_msgs::GetModelState>(
              "/gazebo/get_model_state");
        bool gazebo_available = 
          get_gazebo_model_client_.waitForExistence(ros::Duration(30));

        if (!gazebo_available) {
          ROS_FATAL_STREAM("ClingoGazeboHandler: Gazebo is NOT AVAILABLE");
          throw 
            std::runtime_error("ClingoGazeboHandler: Gazebo is NOT AVAILABLE");
        }

        set_gazebo_model_client_ =
          nh.serviceClient<gazebo_msgs::SetModelState>(
              "/gazebo/set_model_state");
        set_gazebo_model_client_.waitForExistence();

        spawn_model_client_ =
          nh.serviceClient<gazebo_msgs::SpawnModel>(
              "/gazebo/spawn_urdf_model");
        set_gazebo_model_client_.waitForExistence();

        // Spawn all necessary doors
        door_open_status_.resize(doors_.size());
        for (unsigned i = 0; i < doors_.size(); ++i) {
          spawnObject(true, i);
          door_open_status_[i] = false;
        }

        // Spawn 30 obstacle objects
        num_obstacles_ = 0;
        // for (unsigned i = 0; i < 30; ++i) {
        //   spawnObject(false, i);
        // }
      }

      geometry_msgs::Pose getDefaultLocation(bool is_door, int index) {
        geometry_msgs::Pose retval;
        retval.position.y = 500.0f + index * 2;
        retval.position.z = 0.0f;
        retval.orientation.x = 0.0f;
        retval.orientation.y = 0.0f;
        retval.orientation.z = 0.0f;
        retval.orientation.w = 1.0f;
        if (is_door) {
          retval.position.x = 500.0f;
        } else {
          retval.position.x = 600.0f;
        }
        return retval;
      }

      float getDoorWidth(int index) {
        return (0.75f/0.9f) * doors_[index].width;
      }

      geometry_msgs::Pose getDoorLocation(int index) {
        geometry_msgs::Pose retval;

        topological_mapper::Point2f door_center = 0.5 *
          (doors_[index].approach_points[0] +
           doors_[index].approach_points[1]);
        retval.position.x = door_center.x;
        retval.position.y = door_center.y;
        retval.position.z = 0;

        topological_mapper::Point2f diff = 
          (doors_[index].approach_points[0] -
           doors_[index].approach_points[1]);
        float door_yaw = atan2f(diff.y, diff.x);
        retval.orientation = tf::createQuaternionMsgFromYaw(door_yaw);

        return retval;
      }

      void openDoor(int index) {
        if (door_open_status_[index]) 
          return;
        std::string prefix = "auto_door_";
        std::string model_name = prefix +
          boost::lexical_cast<std::string>(index);
        geometry_msgs::Pose pose = getDefaultLocation(true, index);
        teleportEntity(model_name, pose);
        door_open_status_[index] = true;
      }

      void openAllDoors() {
        for (unsigned i = 0; i < doors_.size(); ++i) {
          openDoor(i);
        }
      }

      void closeDoor(int index) {
        if (!door_open_status_[index]) 
          return;
        ROS_INFO_STREAM("Closing door " << index);
        std::string prefix = "auto_door_";
        std::string model_name = prefix +
          boost::lexical_cast<std::string>(index);
        geometry_msgs::Pose pose = getDoorLocation(index);
        teleportEntity(model_name, pose);
        door_open_status_[index] = false;
      }

      void closeAllDoors() {
        ROS_INFO_STREAM("Closing all doors");
        for (unsigned i = 0; i < doors_.size(); ++i) {
          closeDoor(i);
        }
      }

      bool isDoorOpen(int index) {
        return door_open_status_[index];
      }

      void closeAllDoorsFarAwayFromPoint(
          const geometry_msgs::Pose& point, float distance = 2.0) {
        for (unsigned i = 0; i < doors_.size(); ++i) {
          if (!door_open_status_[i])
            continue;
          bool is_door_near = 
            checkClosePoses(point, getDoorLocation(i), distance, false);
          if (!is_door_near) 
            closeDoor(i);
        }
      }

      bool checkClosePoses(const geometry_msgs::Pose& p1,
          const geometry_msgs::Pose& p2, float threshold = 0.05,
          bool check_yaw = true) {
        float dist_diff = 
          sqrtf(pow((p1.position.x - p2.position.x), 2) +
              pow((p1.position.y - p2.position.y), 2));
        if (dist_diff > threshold) {
          return false;
        }
        double yaw1 = tf::getYaw(p1.orientation);
        double yaw2 = tf::getYaw(p2.orientation);
        if (check_yaw && fabs(yaw1 - yaw2) > 0.1) {
          return false;
        }
        return true;
      }

      bool teleportEntity(const std::string& entity,
          const geometry_msgs::Pose& pose) {

        int count = 0;
        int attempts = 5;
        bool location_verified = false;
        while (count < attempts and !location_verified) {
          gazebo_msgs::GetModelState get_srv;
          get_srv.request.model_name = entity;
          get_gazebo_model_client_.call(get_srv);
          location_verified = checkClosePoses(get_srv.response.pose, pose);
          if (!location_verified) {
            gazebo_msgs::SetModelState set_srv;
            set_srv.request.model_state.model_name = entity;
            set_srv.request.model_state.pose = pose;
            set_gazebo_model_client_.call(set_srv);
            if (!set_srv.response.success) {
              ROS_WARN_STREAM("SetModelState service call failed for " << entity
                  << " to " << pose);
            }
          }
          ++count;
        }
        if (!location_verified) {
          ROS_ERROR_STREAM("Unable to teleport " << entity << " to " << pose
              << " despite " << attempts << " attempts.");
          return false;
        }
        return true;
      }

      void spawnObject(bool is_door, int index = 0) {

        gazebo_msgs::SpawnModel spawn;
        std::string prefix;
        if (is_door) {
          prefix = "auto_door_";
          spawn.request.model_xml = boost::regex_replace(door_urdf_, 
              boost::regex("@WIDTH@"),
              boost::lexical_cast<std::string>(getDoorWidth(index)));
          spawn.request.initial_pose = getDoorLocation(index);
        } else {
          prefix = "auto_obs_";
          index = num_obstacles_;
          spawn.request.model_xml = obstacle_urdf_;
          spawn.request.initial_pose = getDefaultLocation(false, index);
        }

        spawn.request.model_name = prefix +
          boost::lexical_cast<std::string>(index);

        if (spawn_model_client_.call(spawn)) {
          if (spawn.response.success) {
            ++num_obstacles_;
            return;
          }
          ROS_WARN_STREAM("Received error message while spawning object: " <<
              spawn.response.status_message);
        }

        ROS_ERROR_STREAM("Unable to spawn: " << spawn.request.model_name);
      }      

    private:

      std::vector<clingo_interface::Door> doors_;
      std::vector<std::string> locations_;
      std::vector<int32_t> location_map_;

      boost::shared_ptr <topological_mapper::MapLoader> mapper_;
      nav_msgs::MapMetaData info_;

      std::vector<bool> door_open_status_;
      std::set<int> obstacles_in_use;
      std::set<int> unused_obstacles_;
      unsigned int num_obstacles_; // obstacles + doors

      ros::ServiceClient get_gazebo_model_client_;
      ros::ServiceClient set_gazebo_model_client_;
      ros::ServiceClient spawn_model_client_;

      std::string obstacle_urdf_;
      std::string door_urdf_;
  };
}

#endif /* end of include guard: GAZEBO_HANDLER_E8TY9EDI */
