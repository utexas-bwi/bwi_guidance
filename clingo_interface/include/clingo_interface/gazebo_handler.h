#ifndef GAZEBO_HANDLER_E8TY9EDI
#define GAZEBO_HANDLER_E8TY9EDI

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
        for (unsigned i = 0; i < doors_.size(); ++i) {
          spawnObject(true, i);
        }

        // Spawn 30 obstacle objects
        num_obstacles_ = 0;
        for (unsigned i = 0; i < 30; ++i) {
          spawnObject(false, i);
        }
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

      void spawnObject(bool is_door, int index = 0) {

        gazebo_msgs::SpawnModel spawn;
        std::string prefix;
        if (is_door) {
          prefix = "auto_door_";
          spawn.request.model_xml = door_urdf_;
        } else {
          prefix = "auto_obs_";
          index = num_obstacles_;
          spawn.request.model_xml = obstacle_urdf_;
        }

        spawn.request.model_name = prefix +
          boost::lexical_cast<std::string>(index);
        spawn.request.initial_pose = getDefaultLocation(is_door, index);

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
