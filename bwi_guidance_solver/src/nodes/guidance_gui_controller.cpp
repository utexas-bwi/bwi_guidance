#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <bwi_msgs/QuestionDialog.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_datatypes.h>

#include <bwi_guidance_msgs/MultiRobotNavigationAction.h>
#include <bwi_guidance_msgs/UpdateGuidanceGui.h>

int system_state = bwi_guidance_msgs::UpdateGuidanceGuiRequest::DISABLE_EPISODE_START;

std::vector<std::string> goal_names;
std::vector<int> goal_graph_ids;
boost::shared_ptr<boost::thread> episode_start_thread;
boost::shared_ptr<actionlib::SimpleActionClient<bwi_guidance_msgs::MultiRobotNavigationAction> > mrn_client;
ros::ServiceClient gui_service;

std::string tf_prefix;
std::string robot_name;
ros::Publisher image_publisher;
cv::Mat u_turn_image, up_arrow_image;
geometry_msgs::Pose robot_location;

void monitorEpisodeStartThread() {
  bwi_msgs::QuestionDialog srv;
  srv.request.type = bwi_msgs::QuestionDialogRequest::CHOICE_QUESTION;
  srv.request.message = "Where would you like to go?";
  BOOST_FOREACH(const std::string& name, goal_names) {
    srv.request.options.push_back(name);
  }
  srv.request.timeout = bwi_msgs::QuestionDialogRequest::NO_TIMEOUT;
  if (gui_service.call(srv)) {
    if (srv.response.index >= 0) {
      ROS_INFO_NAMED("guidance_gui_controller", "req timed out, sending ep start request.");
      bwi_guidance_msgs::MultiRobotNavigationGoal goal;
      goal.goal_node_id = goal_graph_ids[srv.response.index];
      mrn_client->sendGoal(goal);
    } else {
      // Do nothing. The request probably got preempted.
    }
  } else {
    ROS_INFO_NAMED("guidance_gui_controller", "request failed for unspecified reasons.");
  }
}

void displayMessage(const std::string& msg) {
  bwi_msgs::QuestionDialog srv;
  srv.request.type = bwi_msgs::QuestionDialogRequest::DISPLAY;
  srv.request.message = msg;
  srv.request.timeout = bwi_msgs::QuestionDialogRequest::NO_TIMEOUT;
  gui_service.call(srv);
}

void displayImage(const cv::Mat& image) {
  cv_bridge::CvImage out_image;
  out_image.header.frame_id = "laptop_screen_link";
  if (tf_prefix != "") {
    out_image.header.frame_id = tf_prefix + "laptop_screen_link";
  }
  out_image.header.stamp = ros::Time::now();
  out_image.encoding = sensor_msgs::image_encodings::BGR8;
  out_image.image = image;
  image_publisher.publish(out_image.toImageMsg());
}

void clearImage() {
  cv::Mat blank_image = cv::Mat::zeros(120, 160, CV_8UC3);
  if (!robot_name.empty()) {
    cv::putText(blank_image, ("Hi! I'm " + robot_name), cv::Point2f(10, 25),
                CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 215, 255), 2);
  }
  displayImage(blank_image);
}

void showArrowToDestination(const geometry_msgs::Pose &orientation_destination) {

  cv::Mat image;

  // First figure out what exact orientation to show on the screen based on the location of the robot, the orientation
  // of the robot and the destination which the robot wants to indicate.
  float destination_yaw = atan2(orientation_destination.position.y - robot_location.position.y,
                                orientation_destination.position.x - robot_location.position.x);
  float orientation = destination_yaw - tf::getYaw(robot_location.orientation);
  while (orientation <= -M_PI) orientation += 2 * M_PI;
  while (orientation > M_PI) orientation -= 2 * M_PI;

  cv::Mat rotated_image;
  int height = u_turn_image.rows, width = u_turn_image.cols;
  if (fabs(orientation) > 5.0 * M_PI / 6.0) {
    rotated_image = u_turn_image;
  } else {
    height = up_arrow_image.rows, width = up_arrow_image.cols;
    cv::Point2f center(width/2, height/2);
    cv::Mat rotation_matrix =
      cv::getRotationMatrix2D(center, orientation * 180 / M_PI, 1.0);

    cv::warpAffine(up_arrow_image, rotated_image, rotation_matrix,
                   cv::Size(width, height));
  }

  float height_ratio = 119.0 / height;
  float width_ratio = 159.0 / width;
  float min_ratio = std::min(height_ratio, width_ratio);

  cv::Mat resized_image;
  cv::resize(rotated_image, resized_image,
             cv::Size(0,0), min_ratio, min_ratio);

  image = cv::Mat::zeros(120, 160, CV_8UC3);
  int top = (image.rows - resized_image.rows) / 2;
  int bottom = image.rows - resized_image.rows - top;
  int left = (image.cols - resized_image.cols) / 2;
  int right = image.cols - resized_image.cols - left;

  cv::copyMakeBorder(resized_image, image, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
  displayImage(image);

}

void showFollowMeImage() {
  cv::Mat image = cv::Mat::zeros(120, 160, CV_8UC3);
  if (!robot_name.empty()) {
    cv::putText(image, ("Hi! I'm " + robot_name), cv::Point2f(10, 25),
                CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 215, 255), 2);
  }
  cv::putText(image, "Follow Me!", cv::Point2f(10, image.rows/2-5),
              CV_FONT_HERSHEY_SIMPLEX, 0.90, cv::Scalar(0, 215, 255), 4);
  displayImage(image);
}

void showPleaseWaitImage() {
  cv::Mat image = cv::Mat::zeros(120, 160, CV_8UC3);
  if (!robot_name.empty()) {
    cv::putText(image, ("Hi! I'm " + robot_name), cv::Point2f(10, 25),
                CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 215, 255), 2);
  }
  cv::putText(image, "Please Wait!", cv::Point2f(10, image.rows/2-5),
              CV_FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 215, 255), 3);
  displayImage(image);
}

bool updateGui(bwi_guidance_msgs::UpdateGuidanceGui::Request& request,
               bwi_guidance_msgs::UpdateGuidanceGui::Response& response) {
  response.success = true;
  response.message = "";
  /* ROS_INFO_STREAM_NAMED("guidance_gui_controller", "received request type: " << request.type); */
  switch(request.type) {
    case bwi_guidance_msgs::UpdateGuidanceGuiRequest::ENABLE_EPISODE_START:
      if (system_state == bwi_guidance_msgs::UpdateGuidanceGuiRequest::ENABLE_EPISODE_START) {
        response.success = false;
        response.message = "Episode start already enabled!";
      } else {
        clearImage();
        system_state = bwi_guidance_msgs::UpdateGuidanceGuiRequest::ENABLE_EPISODE_START;
        episode_start_thread.reset(new boost::thread(&monitorEpisodeStartThread));
      }
      break;
    case bwi_guidance_msgs::UpdateGuidanceGuiRequest::DISABLE_EPISODE_START:
      clearImage();
      displayMessage("If you would like navigation assistance, please follow me till I stop and let me know.");
      system_state = bwi_guidance_msgs::UpdateGuidanceGuiRequest::DISABLE_EPISODE_START;
      break;
    case bwi_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_NOTHING:
      clearImage();
      displayMessage("");
      system_state = bwi_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_NOTHING;
      break;
    case bwi_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_ORIENTATION:
      showArrowToDestination(request.orientation_destination);
      displayMessage("Please move ahead in the indicated direction!");
      system_state = bwi_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_ORIENTATION;
      break;
    case bwi_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_FOLLOWME:
      showFollowMeImage();
      displayMessage("Follow Me!");
      system_state = bwi_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_FOLLOWME;
      break;
    case bwi_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_PLEASEWAIT:
      showPleaseWaitImage();
      displayMessage("Please wait!");
      system_state = bwi_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_FOLLOWME;
      break;
  };

  return true;
}

void locationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose) {
  robot_location = pose->pose.pose;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "guidance_gui_controller");
  ros::NodeHandle nh, private_nh("~");

  // TODO paramtrize
  goal_names.push_back("top-left");
  goal_graph_ids.push_back(27);
  goal_names.push_back("top-right");
  goal_graph_ids.push_back(23);
  goal_names.push_back("bottom-left");
  goal_graph_ids.push_back(42);
  goal_names.push_back("bottom-right");
  goal_graph_ids.push_back(8);
  goal_names.push_back("center");
  goal_graph_ids.push_back(39);

  // Read images from parameters
  std::string up_arrow_image_file, u_turn_image_file;
  std::string images_dir = ros::package::getPath("bwi_guidance") + "/images";

  std::string key;
  tf_prefix = "";
  if (private_nh.searchParam("tf_prefix", key)) {
    private_nh.getParam(key, tf_prefix);
  }

  if (tf_prefix != "" && tf_prefix[tf_prefix.size() - 1] != '/') {
    tf_prefix = tf_prefix + "/";
  }

  robot_name = tf_prefix.substr(0, tf_prefix.size() - 1);
  if (!robot_name.empty()) {
    robot_name[0] = toupper(robot_name[0]);
  }

  private_nh.param<std::string>("up_arrow_image", up_arrow_image_file, images_dir + "/Up.png");
  up_arrow_image = cv::imread(up_arrow_image_file);
  private_nh.param<std::string>("u_turn_image", u_turn_image_file, images_dir + "/UTurn.png");
  u_turn_image = cv::imread(u_turn_image_file);

  mrn_client.reset(new actionlib::SimpleActionClient<bwi_guidance_msgs::MultiRobotNavigationAction>("/guidance", true));
  ROS_INFO_NAMED("guidance_gui_controller", "Waiting for guidance action server.");
  mrn_client->waitForServer();
  ROS_INFO_NAMED("guidance_gui_controller", "Guidance action server found.");

  ros::ServiceServer enable_episode_start_service = nh.advertiseService("update_gui", &updateGui);
  gui_service = nh.serviceClient<bwi_msgs::QuestionDialog>("question_dialog");

  ROS_INFO_NAMED("guidance_gui_controller", "Waiting for segbot_gui service.");
  gui_service.waitForExistence();
  ROS_INFO_NAMED("guidance_gui_controller", "segbot_gui service found.");


  image_publisher = nh.advertise<sensor_msgs::Image>("image", 1);
  ros::Subscriber robot_location_subscriber = nh.subscribe("amcl_pose", 1, locationHandler);

  ros::spin();

  return 0;
}
