#include <algorithm>
#include <boost/foreach.hpp>
#include <cstdlib>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv/highgui.h>
#include <tf/transform_datatypes.h>
#include <bwi_exp1/base_robot_positioner.h>
#include <topological_mapper/map_inflator.h>
#include <bwi_msgs/RobotInfoArray.h>
#include <boost/range/adaptor/map.hpp>

namespace bwi_exp1 {

  BaseRobotPositioner::BaseRobotPositioner(
      boost::shared_ptr<ros::NodeHandle>& nh) : 
    nh_(nh), robot_screen_publisher_(nh) {
    
    // Read images from parameters
    std::string up_arrow_image_file;
    std::string images_dir = ros::package::getPath("bwi_exp1") + "/images";
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("up_arrow_image", up_arrow_image_file,
        images_dir + "/Up.png");
    up_arrow_ = cv::imread(up_arrow_image_file);
    blank_image_ = cv::Mat::zeros(120, 160, CV_8UC3);

    // Read all other parameters
    std::string map_file, graph_file, robot_file, experiment_file;
    double robot_radius, robot_padding;
    if (!private_nh.getParam("map_file", map_file)) {
      ROS_FATAL_STREAM("RobotPosition: ~map_file parameter required");
      exit(-1);
    }
    if (!private_nh.getParam("graph_file", graph_file)) {
      ROS_FATAL_STREAM("RobotPosition: ~graph_file parameter required");
      exit(-1);
    }
    if (!private_nh.getParam("default_robot_file", robot_file)) {
      ROS_FATAL_STREAM("RobotPosition: ~default_robot_file parameter required");
      exit(-1);
    }
    if (!private_nh.getParam("experiment_file", experiment_file)) {
      ROS_FATAL_STREAM("RobotPosition: ~experiment_file parameter required");
      exit(-1);
    }
    private_nh.param<double>("robot_radius", robot_radius, 0.25);
    private_nh.param<double>("robot_padding", robot_padding, 0.1);
    private_nh.param<double>("search_distance", search_distance_, 0.75);

    // Setup map, graph and robots
    nav_msgs::OccupancyGrid uninflated_map;
    mapper_.reset(new topological_mapper::MapLoader(map_file));
    mapper_->getMapInfo(map_info_);
    mapper_->getMap(uninflated_map);
   
    topological_mapper::inflateMap(robot_radius + robot_padding, 
        uninflated_map, map_);
    topological_mapper::readGraphFromFile(graph_file, map_info_, graph_);
    
    readDefaultRobotsFromFile(robot_file, default_robots_);
    BOOST_FOREACH(const Robot& robot, default_robots_.robots) {
      robot_screen_publisher_.addRobot(robot.id);
      robot_locations_[robot.id] = 
        convert2dToPose(robot.default_loc.x, robot.default_loc.y, 0);
      assigned_robot_locations_[robot.id] = robot_locations_[robot.id]; 
      robot_ok_[robot.id] = true;
      robot_screen_orientations_[robot.id] = 
        std::numeric_limits<float>::quiet_NaN(); 
    }

    readExperimentFromFile(experiment_file, experiment_);

    private_nh.param<bool>("debug", debug_, false);

    // Setup ros topic callbacks and services
    get_gazebo_model_client_ = nh->serviceClient<gazebo_msgs::GetModelState>(
        "/gazebo/get_model_state");
    gazebo_available_ = 
      get_gazebo_model_client_.waitForExistence(ros::Duration(30));

    set_gazebo_model_client_ = nh->serviceClient<gazebo_msgs::SetModelState>(
        "/gazebo/set_model_state");
    gazebo_available_ &= 
      set_gazebo_model_client_.waitForExistence(ros::Duration(5));

    if (gazebo_available_) {
      ROS_INFO_STREAM("Gazebo is AVAILABLE");
    } else {
      ROS_INFO_STREAM("Gazebo is NOT AVAILABLE");
    }

    odometry_subscriber_ = 
      nh->subscribe("person/odom", 1, &BaseRobotPositioner::odometryCallback, 
          this);
    current_instance_ = -1;
    instance_in_progress_ = false;
    experiment_status_subscriber_ = 
      nh->subscribe("experiment_controller/experiment_status", 
          1, &BaseRobotPositioner::experimentCallback, this);

    position_publisher_ = 
      nh->advertise<bwi_msgs::RobotInfoArray>("robot_positions", 1, true);
  }

  BaseRobotPositioner::~BaseRobotPositioner() {
    if (publishing_thread_) {
      publishing_thread_->join();
    }
  }

  void BaseRobotPositioner::finalizeExperimentInstance() {
    boost::mutex::scoped_lock lock(robot_modification_mutex_);
    BOOST_FOREACH(const Robot& robot, default_robots_.robots) {
      robot_locations_[robot.id] = 
        convert2dToPose(robot.default_loc.x, robot.default_loc.y, 0);
      robot_screen_orientations_[robot.id] = 
        std::numeric_limits<float>::quiet_NaN(); 
    }
  }

  void BaseRobotPositioner::produceDirectedArrow(float orientation,
      cv::Mat& image) {

    int height = up_arrow_.rows, width = up_arrow_.cols;
    cv::Point2f center(width/2, height/2);
    cv::Mat rotation_matrix = 
      cv::getRotationMatrix2D(center, orientation * 180 / M_PI, 1.0);

    cv::Mat rotated_image;
    cv::warpAffine(up_arrow_, rotated_image, rotation_matrix, 
        cv::Size(width, height)); 

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

    cv::copyMakeBorder(resized_image, image, top, bottom, left, right, 
        cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
  }

  void BaseRobotPositioner::experimentCallback(
      const bwi_msgs::ExperimentStatus::ConstPtr es) {
    if (es->instance_in_progress && es->instance_number != current_instance_) {
      startExperimentInstance(es->instance_name);
      current_instance_ = es->instance_number;
      instance_in_progress_ = true;
    } else if (!es->instance_in_progress && instance_in_progress_) {
      finalizeExperimentInstance();
      instance_in_progress_ = false;
    }
  }

  geometry_msgs::Pose BaseRobotPositioner::convert2dToPose(
      float x, float y, float yaw) {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0;
    pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    return pose;
  }

  bool BaseRobotPositioner::checkClosePoses(const geometry_msgs::Pose& p1,
      const geometry_msgs::Pose& p2) {
    if (fabs(p1.position.x - p2.position.x) > 0.1 ||
        fabs(p1.position.y - p2.position.y) > 0.1) {
      return false;
    }
    double yaw1 = tf::getYaw(p1.orientation);
    double yaw2 = tf::getYaw(p2.orientation);
    if (fabs(yaw1 - yaw2) > 0.1) {
      return false;
    }
    return true;
  }

  bool BaseRobotPositioner::teleportEntity(const std::string& entity,
      const geometry_msgs::Pose& pose) {

    if (!gazebo_available_) {
      ROS_ERROR_STREAM("Teleportation requested, but gazebo unavailable");
      return false;
    }

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
    }
  }

  geometry_msgs::Pose BaseRobotPositioner::positionRobot(
      const topological_mapper::Point2f& from,
      const topological_mapper::Point2f& at,
      const topological_mapper::Point2f& to) {

    geometry_msgs::Pose resp;
    topological_mapper::Point2f from_map = 
      topological_mapper::toMap(from, map_info_);
    topological_mapper::Point2f at_map = 
      topological_mapper::toMap(at, map_info_);

    // Figure out if you want to stay on the outside angle or not
    bool use_outside_angle = true;
    float yaw1 = -atan2((to - at).y, (to - at).x);
    float yaw2 = -atan2((from - at).y, (from - at).x);

    topological_mapper::Point2f yaw1_pt(cosf(yaw1),sinf(yaw1));
    topological_mapper::Point2f yaw2_pt(cosf(yaw2),sinf(yaw2));
    topological_mapper::Point2f yawmid_pt = 0.5 * (yaw1_pt + yaw2_pt);

    if (topological_mapper::getMagnitude(yawmid_pt) < 0.1) {
      use_outside_angle = false;
    }

    size_t y_test = at.y - search_distance_ / map_info_.resolution;
    float location_fitness = -1;
    topological_mapper::Point2f test_coords;
    topological_mapper::Point2f map_coords;

    while(y_test < at.y + search_distance_ / map_info_.resolution) {
      size_t x_test = at.x - search_distance_ / map_info_.resolution;
      while (x_test < at.x + search_distance_ / map_info_.resolution) {

        topological_mapper::Point2f test(x_test, y_test);
        
        // Check if x_test, y_test is free.
        size_t map_idx = MAP_IDX(map_info_.width, x_test, y_test);
        if (map_.data[map_idx] != 0) {
          x_test++;
          continue;
        }

        // Check if it is on the outside or not
        if (use_outside_angle) {
          if ((from + to - 2 * at).dot(test - at) > 0) {
            x_test++;
            continue;
          }
        }

        topological_mapper::Point2f test_loc(x_test, y_test);

        float dist1 = 
          topological_mapper::minimumDistanceToLineSegment(from, at, test_loc);
        float dist2 = 
          topological_mapper::minimumDistanceToLineSegment(at, to, test_loc);
        float fitness = std::min(dist1, dist2);

        if (fitness > location_fitness) {
          test_coords = test_loc;
          map_coords = topological_mapper::toMap(test_loc, map_info_);
          resp.position.x = map_coords.x;
          resp.position.y = map_coords.y;
          location_fitness = fitness;
        }

        x_test++; 
      }
      y_test++;
    }

    // Calculate yaw
    yaw1 = atan2(resp.position.y - at_map.y, resp.position.x - at_map.x); 
    yaw2 = atan2(resp.position.y - from_map.y, resp.position.x - from_map.x);

    yaw1_pt = topological_mapper::Point2f(cosf(yaw1),sinf(yaw1));
    yaw2_pt = topological_mapper::Point2f(cosf(yaw2),sinf(yaw2));
    yawmid_pt = 0.5 * (yaw1_pt + yaw2_pt);

    float resp_yaw = atan2f(yawmid_pt.y, yawmid_pt.x); 
    resp.orientation = tf::createQuaternionMsgFromYaw(resp_yaw);

    if (debug_) {
      cv::Mat image;
      mapper_->drawMap(image, map_);

      cv::circle(image, from, 5, cv::Scalar(255, 0, 0), -1);
      cv::circle(image, at, 5, cv::Scalar(255, 0, 255), 2);
      cv::circle(image, to, 5, cv::Scalar(0, 0, 255), -1);
      cv::circle(image, test_coords, 5, cv::Scalar(0, 255, 0), 2);

      cv::circle(image, test_coords + 
          topological_mapper::Point2f(20 * cosf(resp_yaw), 20 * sinf(resp_yaw)), 
          3, cv::Scalar(0, 255, 0), -1);

      cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);
      cv::imshow("Display window", image);
      cv::waitKey(0);
    }

    return resp;
  }

  void BaseRobotPositioner::start() {
    publishing_thread_.reset(
        new boost::thread(boost::bind(&BaseRobotPositioner::run, this)));
  }

  void BaseRobotPositioner::run() {
    ROS_INFO_STREAM("BaseRobotPositioner starting up...");
    robot_screen_publisher_.start();
    ros::Rate rate(10);
    bool first = true;
    while (ros::ok()) {
      bool change = first;
      BOOST_FOREACH(const std::string& robot_id, 
          robot_locations_ | boost::adaptors::map_keys) {
        bool already_there = checkClosePoses(robot_locations_[robot_id],
            assigned_robot_locations_[robot_id]);
        if (!already_there) {
          change = true;
          robot_ok_[robot_id] = 
            teleportEntity(robot_id, robot_locations_[robot_id]);
          assigned_robot_locations_[robot_id] = robot_locations_[robot_id];
        }
      }
      if (change) {
        bwi_msgs::RobotInfoArray out_msg;
        out_msg.header.frame_id = "map";
        out_msg.header.stamp = ros::Time::now();
        BOOST_FOREACH(const std::string& robot_id, 
            robot_locations_ | boost::adaptors::map_keys) {
          bwi_msgs::RobotInfo robot_info;
          robot_info.pose = assigned_robot_locations_[robot_id];
          robot_info.direction = robot_screen_orientations_[robot_id];
          robot_info.is_ok = robot_ok_[robot_id];
          out_msg.robots.push_back(robot_info);
        }
        position_publisher_.publish(out_msg);
      }
      first = false;
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO_STREAM("BaseRobotPositioner shutting down...");
  }
  
} /* bwi_exp1 */            
