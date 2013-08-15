#include <algorithm>
#include <cstdlib>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv/highgui.h>
#include <tf/transform_datatypes.h>
#include <bwi_exp1/base_robot_positioner.h>
#include <topological_mapper/map_inflator.h>

namespace bwi_exp1 {

  BaseRobotPositioner::BaseRobotPositioner(
      boost::shared_ptr<ros::NodeHandle>& nh) : nh_(nh) {
    
    // Get private parameters
    std::string up_arrow_image_file;
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("up_arrow_image", up_arrow_image_file,
        ros::package::getPath("bwi_exp1") + "/images/Up.png");
    up_arrow_ = cv::imread(up_arrow_image_file);

    std::string map_file, graph_file;
    double robot_radius, robot_padding;
    if (!private_nh.getParam("map_file", map_file)) {
      ROS_FATAL_STREAM("RobotPosition: ~map_file parameter required");
      exit(-1);
    }
    if (!private_nh.getParam("graph_file", graph_file)) {
      ROS_FATAL_STREAM("RobotPosition: ~graph_file parameter required");
      exit(-1);
    }
    private_nh.param<double>("robot_radius", robot_radius, 0.25);
    private_nh.param<double>("robot_padding", robot_padding, 0.1);
    private_nh.param<double>("search_distance", search_distance_, 0.75);

    nav_msgs::OccupancyGrid uninflated_map;
    mapper_.reset(new topological_mapper::MapLoader(map_file));
    mapper_->getMapInfo(map_info_);
    mapper_->getMap(uninflated_map);
   
    topological_mapper::inflateMap(robot_radius + robot_padding, 
        uninflated_map, map_);
    topological_mapper::readGraphFromFile(graph_file, map_info_, graph_);
    
    private_nh.param<bool>("debug", debug_, false);

    // Setup ros topic callbacks and services
    get_gazebo_model_client_ = nh->serviceClient<gazebo_msgs::GetModelState>(
        "/gazebo/get_model_state");
    bool gazebo_available_ = 
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

        # Setup components required for publishing directed arrows to the
        # robot screens
        images_dir = roslib.packages.get_pkg_dir("bwi_exp1") + "/images"
        self.arrow = readImage(images_dir + "/Up.png")
        self.image_none = numpy.zeros((120,160,3), numpy.uint8)

        self.robot_image_publisher = RobotScreenPublisher()
        self.robots = self.experiment['robots']
        for i, robot in enumerate(self.robots):
            robot_id = robot['id']
            self.robot_image_publisher.add_robot(robot_id)

        # Track robots currently being used in an experiment instance 
        self.robots_in_instance = []
        self.robot_locations = []
        self.robot_images = []

class RobotScreenPublisher(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.robot_image_publisher = {}
        self.robot_image = {}
        self.robot_image_lock = {}
        self.bridge = CvBridge()

    def add_robot(self, robotid):
        self.robot_image_publisher[robotid] = rospy.Publisher(
                robotid + '/image', Image)
        self.robot_image[robotid] = numpy.zeros((120,160,3), numpy.uint8)
        self.robot_image_lock[robotid] = threading.Lock()

    def update(self, robotid, new_image):
        self.robot_image_lock[robotid].acquire()
        self.robot_image[robotid] = new_image
        self.robot_image_lock[robotid].release()

    def run(self):
        rate = bwi_tools.WallRate(5) # 5hz
        try:
            while not rospy.is_shutdown():
                for robotid in self.robot_image_publisher.keys():
                    self.robot_image_lock[robotid].acquire()
                    image = self.robot_image[robotid]
                    cv_image = cv.fromarray(image)
                    image_msg = self.bridge.cv_to_imgmsg(cv_image, "bgr8")
                    self.robot_image_lock[robotid].release()
                    self.robot_image_publisher[robotid].publish(image_msg)
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
        rospy.loginfo("Robot screen publisher thread shutting down...")

  }

  BaseRobotPositioner::~BaseRobotPositioner() {}

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
  
} /* bwi_exp1 */            
