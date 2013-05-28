/*
 * A generic object controller plugin. Based on the diffdrive plugin 
 * developed for the erratic robot (see copyright notice below). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins
 *
 * Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

#ifndef OBJECT_CONTROLLER_PLUGIN_HH
#define OBJECT_CONTROLLER_PLUGIN_HH

#include <map>

#include <common/common.hh>
#include <physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <bwi_msgs/UpdatePluginState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{
class Joint;
class Entity;

class ObjectControllerPlugin : public ModelPlugin
{
  public: ObjectControllerPlugin();
  public: ~ObjectControllerPlugin();
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  protected: virtual void UpdateChild();
  protected: virtual void FiniChild();

private:
  void writePositionData(double step_time);
  void publishOdometry(double step_time);
  void getWheelVelocities();

  physics::WorldPtr world;
  physics::ModelPtr parent;
  event::ConnectionPtr updateConnection;

  math::Pose last_odom_pose_;

  // ROS STUFF
  ros::NodeHandle* rosnode_;
  ros::Publisher pub_, pub2_;
  ros::Subscriber sub_, sub2_;
  tf::TransformBroadcaster *transform_broadcaster_;
  nav_msgs::Odometry odom_;
  std::string tf_prefix_;
  ros::ServiceServer update_state_service_server_;

  boost::mutex lock;
  boost::mutex state_lock_;

  bool pause_;

  double timeout_period_;
  common::Time time_of_last_message_;

  std::string modelNamespace;
  std::string topicName;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();

  // DiffDrive stuff
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
  bool updateState(bwi_msgs::UpdatePluginState::Request &req,
    bwi_msgs::UpdatePluginState::Response &resp);

  double x_;
  double y_;
  double rot_;
  bool alive_;

  // Update Rate
  double updateRate;
  double update_period_;
  common::Time last_update_time_;

  // Simple map stuff
  std::string mapTopic;
  std::string globalFrame;
  double modelRadius;
  double modelPadding;
  double circumscribed_model_distance_;
  nav_msgs::OccupancyGrid map_;
  bool map_available_;
  void getSimpleMap(const nav_msgs::OccupancyGrid::ConstPtr& map);

};

}

#endif

