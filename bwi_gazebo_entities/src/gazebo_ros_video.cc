/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */

#include <bwi_gazebo_entities/gazebo_ros_video.h>

namespace gazebo
{


////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosVideo::GazeboRosVideo()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosVideo::~GazeboRosVideo()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosVideo::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf )
{
  this->model = _parent;
  this->updateConnection = event::Events::ConnectPreRender(boost::bind(&GazeboRosVideo::UpdateChild, this));

  if (!_sdf->HasElement("robotNamespace")) {
    ROS_WARN("Object controller plugin missing <robotNamespace>, defaults to \"\"");
    this->modelNamespace = "";
  } else {
    this->modelNamespace = _sdf->GetElement("robotNamespace")->GetValueString();
  }

  if (!_sdf->HasElement("topicName")) {
    ROS_WARN("Object controller plugin missing <topicName>, defaults to image_raw");
    this->topicName = "image_raw";
  } else {
    this->topicName = _sdf->GetElement("topicName")->GetValueString();
  }

  if (!_sdf->HasElement("height")) {
    ROS_WARN("Object controller plugin missing <height>, defaults to 480");
    this->height = 240;
  } else {
    this->height = _sdf->GetElement("height")->GetValueInt();
  }

  if (!_sdf->HasElement("width")) {
    ROS_WARN("Object controller plugin missing <width>, defaults to 640");
    this->width = 320;
  } else {
    this->width = _sdf->GetElement("width")->GetValueInt();
  }

  std::string _name = "laptop_visual";
  video_visual_.reset(new VideoVisual(_name, _parent, this->height, this->width));

  //TODO: Do not use global queue. Remove Image Transport usage here

  // Initialize the ROS node and subscribe to cmd_vel
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "video_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_.reset(new ros::NodeHandle(this->modelNamespace));
  it_.reset(new image_transport::ImageTransport(*rosnode_));
  camera_subscriber_ = it_->subscribe(this->topicName, 1, &GazeboRosVideo::processImage, this);
  ROS_INFO("Starting gazebo ROS video plugin in ns: %s", this->modelNamespace.c_str());
  new_image_available_ = false;

  this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosVideo::QueueThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosVideo::UpdateChild()
{
  boost::mutex::scoped_lock scoped_lock(m_image_);
  if (new_image_available_) {
    video_visual_->render(image_->image);
  }
  new_image_available_ = false;
}

void GazeboRosVideo::processImage(const sensor_msgs::ImageConstPtr &msg) {
  // Get a reference to the image from the image message pointer
  boost::mutex::scoped_lock scoped_lock(m_image_);
  image_ = cv_bridge::toCvCopy(msg, "bgr8");
  new_image_available_ = true;
}

void GazeboRosVideo::QueueThread()
{
  ros::Rate r(10);
  while (rosnode_->ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}

GZ_REGISTER_VISUAL_PLUGIN(GazeboRosVideo);

}
