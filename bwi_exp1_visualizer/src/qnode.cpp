/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <bwi_exp1_visualizer/qnode.hpp>

#include <boost/algorithm/string/join.hpp>
#include <cstdlib>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace bwi_exp1_visualizer {

  /*****************************************************************************
   ** Implementation
   *****************************************************************************/

  QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
  {}

  QNode::~QNode() {
    if (ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
  }

  bool QNode::init() {
    ros::init(init_argc,init_argv,"bwi_exp1_visualizer");
    ros::start(); // explicitly needed since nodehandle going out of scope

    // Get the robot positioner service
    ros::NodeHandle nh;
    // robot_positioner_ = nh.serviceClient<bwi_msgs::PositionRobot>("position");
    // ROS_INFO_STREAM("Waiting for service: /position");
    // robot_positioner_->waitForExistence();
    // ROS_INFO_STREAM("Service Acquired: /position");

    // Get parameters to understand image generation
    ros::param::get("~map_file", map_file_);
    ros::param::get("~graph_file", graph_file_);
    ros::param::get("~experiment_file", experiment_file_);
    ros::param::get("~data_directory", data_directory_);
    ros::param::get("~users_file", users_file_);

    /* Initialize the map and associated data now the we know where the map 
       file is */
    mapper_.reset(new topological_mapper::MapLoader(map_file_));
    nav_msgs::MapMetaData info;
    mapper_->getMapInfo(info);
    topological_mapper::readGraphFromFile(graph_file_, info, graph_);

    /* Process experiment data */
    bwi_exp1::readExperimentFromFile(experiment_file_, experiments_);
    bwi_exp1::getInstanceNames(experiments_, experiment_box_strings_);
    
    /* Process user data */
    bwi_exp1::readUserDataFromFile(users_file_, users_);

    /* 1st entry is all */
    user_box_strings_.push_back("All");
    user_box_to_idx_map_.push_back(getAllUserIds(users_));

    /* next are the various orderings */
    std::vector< std::vector<std::string> > orderings;
    bwi_exp1::computeOrderings(experiments_, orderings);
    for (size_t i = 0; i < orderings.size(); ++i) {
      user_box_strings_.push_back(boost::algorithm::join(orderings[i], ","));
      user_box_to_idx_map_.push_back(
          getUserIdsForOrdering(users_, orderings[i]));
    }

    /* next add individual users */
    for (size_t i = 0; i < users_.size(); ++i) {
      user_box_strings_.push_back("User: " + users_[i].id);
      std::vector<size_t> user_idx(1, i);
      user_box_to_idx_map_.push_back(user_idx);
    }

    /* select a color for each user */
    for (size_t i = 0; i < users_.size(); ++i) {
      users_[i].color[0] = 64 + rand() % 128;
      users_[i].color[1] = 64 + rand() % 128;
      users_[i].color[2] = 64 + rand() % 128;
    }

    /* Process odometry data */
    for (size_t exp = 0; exp < experiment_box_strings_.size(); ++exp) {
      std::vector< std::vector<bwi_exp1::LocationStamped> >
        all_paths_for_experiment;
      for (size_t u = 0; u < users_.size(); ++u) {
        std::vector<bwi_exp1::LocationStamped> path;
        std::string file = data_directory_ + "/" + users_[u].id + "_" + 
          experiment_box_strings_[exp];
        bwi_exp1::readOdometry(file, path);
        all_paths_for_experiment.push_back(path);
      }
      odometry_.push_back(all_paths_for_experiment);
    }

    /* default values for data */
    time_step_ = 0.1;
    max_experiment_time_ = 0.0;
    max_time_steps_ = 0;
    current_experiment_time_ = 0;
    current_time_step_ = 0;
    autoplay = false;

    start();
    return true;
  }

  void QNode::on_timeSlider_valueChanged(int value) {
    if (current_time_step_ != value) {
      updateFrame(value);
    }
  }

  void QNode::on_autoPlayBox_toggled(bool checked) {
    autoplay = checked;
  }

  void QNode::on_experimentBox_currentIndexChanged(int index) {
    current_experiment_index_ = index;
    current_time_step_ = 0;
    updateFrame(current_time_step_);
  }

  void QNode::on_userBox_currentIndexChanged(int index) {
    current_user_index_ = index;
    updateFrame(current_time_step_);
  }

  void QNode::updateFrame(int current_time_step) {

    // Draw image
    mapper_->drawMap(generated_image_);
    
    // mark the start and goal locations
    bwi_exp1::Instance& exp = 
      bwi_exp1::getInstance(experiments_, current_experiment_index_);
    cv::Vec2f start_loc(exp.start_loc.x, exp.start_loc.y);
    cv::Vec2f goal_loc(exp.ball_loc.x, exp.ball_loc.y);
    cv::circle(generated_image_, toImage(start_loc), 5, cv::Scalar(0,0,255),3);
    cv::circle(generated_image_, toImage(goal_loc), 5, cv::Scalar(128,255,128),3);

    // mark the path
    cv::Point prev_point_loc;
    //std::cout << exp.path.size() << std::endl;
    for (size_t i = 0; i < exp.path.size(); ++i) {
      //std::cout << "in here";
      topological_mapper::Graph::vertex_descriptor vd = 
          boost::vertex(exp.path[i].graph_id, graph_);
      cv::Point point_loc(graph_[vd].location.x, graph_[vd].location.y);
      if (i > 0) {
        cv::line(generated_image_,
            point_loc,
            prev_point_loc,
            cv::Scalar(0,0,0),
            2);
      }
      if (exp.path[i].robot_present) {
        cv::circle(generated_image_, point_loc, 5, cv::Scalar(128,255,128),-1);
        cv::circle(generated_image_, point_loc, 7, cv::Scalar(255,0,0),3);
      } else {
        cv::circle(generated_image_, point_loc, 5, cv::Scalar(0,0,0),-1);
      }
      prev_point_loc = point_loc;
    }

    // mark the extra robots
    for (size_t i = 0; i < exp.extra_robots.size(); ++i) {
      cv::Vec2f point_loc(exp.extra_robots[i].x, exp.extra_robots[i].y);
      cv::circle(generated_image_, 
          toImage(point_loc), 5, cv::Scalar(128,255,128),-1);
      cv::circle(generated_image_, 
          toImage(point_loc), 7, cv::Scalar(255,0,0),3);
    }

    current_experiment_time_ = current_time_step * time_step_;
    std::vector<size_t>& users_in_frame = 
        user_box_to_idx_map_[current_user_index_];
    max_experiment_time_ = 0.0;
    for (size_t u = 0; u < users_in_frame.size(); ++u) {
      std::vector<bwi_exp1::LocationStamped>& path = 
        odometry_[current_experiment_index_][users_in_frame[u]];
      for (size_t t = 1; t < path.size(); ++t) {
        if (path[t].seconds_since_start < 0.1)
          continue;
        if (path[t].seconds_since_start > current_experiment_time_)
          break;
        cv::Vec2f prev_pt(path[t-1].location.x, path[t-1].location.y); 
        cv::Vec2f curr_pt(path[t].location.x, path[t].location.y); 
        cv::line(generated_image_,
            toImage(prev_pt),
            toImage(curr_pt),
            cv::Scalar(
                users_[users_in_frame[u]].color[0], 
                users_[users_in_frame[u]].color[1], 
                users_[users_in_frame[u]].color[2]),
            3);
      }
      if (path.size() != 0) { // Data did not load properly
        max_experiment_time_ = std::max(max_experiment_time_, path[path.size() - 1].seconds_since_start);
        if (path[path.size() - 1].seconds_since_start <= current_experiment_time_) {
          cv::Vec2f curr_pt(path[path.size() - 1].location.x, path[path.size() - 1].location.y); 
          cv::circle(generated_image_,
              toImage(curr_pt), 10,
              cv::Scalar(
                  users_[users_in_frame[u]].color[0], 
                  users_[users_in_frame[u]].color[1], 
                  users_[users_in_frame[u]].color[2]),
              -1);
        }
      }
    }

    current_time_step_ = current_time_step;
    max_time_steps_ = ceil(max_experiment_time_ / time_step_);

    emit updateFrameInfo();
  }

  cv::Point QNode::toImage(const cv::Vec2f& pt) {
    nav_msgs::MapMetaData info;
    mapper_->getMapInfo(info);

    cv::Point image_loc;
    image_loc.x = (pt[0] - info.origin.position.x) / info.resolution;
    image_loc.y = (pt[1] - info.origin.position.y) / info.resolution;
    return image_loc;
  }

  void QNode::run() {
    ros::Rate loop_rate(1.0/time_step_);
    while (ros::ok()) {
      if (autoplay) {
        emit incrementTime();
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
  }

}  // namespace bwi_exp1_visualizer
