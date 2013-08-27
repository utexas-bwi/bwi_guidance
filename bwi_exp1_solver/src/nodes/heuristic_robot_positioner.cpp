#include<fstream>

#include <bwi_exp1_solver/ValueIteration.h>
#include <bwi_exp1_solver/person_estimator.h>
#include <bwi_exp1_solver/person_model.h>
#include <topological_mapper/map_loader.h>
#include <bwi_exp1/base_robot_positioner.h>
#include <tf/transform_datatypes.h>

using namespace bwi_exp1;
using topological_mapper::Graph;

class HeuristicRobotPositioner : public BaseRobotPositioner {

  private:

    int num_robots_available_;
    int goal_idx_;

    std::string instance_name_;
    size_t current_graph_idx_;
    size_t assigned_robots_;

  public:

    HeuristicRobotPositioner(boost::shared_ptr<ros::NodeHandle>& nh) :
        BaseRobotPositioner(nh) {
      ros::NodeHandle private_nh("~");
      private_nh.getParam("goal_idx", goal_idx_);
      private_nh.param("num_robots_available", num_robots_available_, 5);
    }

    virtual ~HeuristicRobotPositioner() {}

    virtual void startExperimentInstance(
        const std::string& instance_name) {

      instance_name_ = instance_name;
      assigned_robots_ = 0;

      const Instance& instance = 
        getInstance(experiment_, instance_name_);

      topological_mapper::Point2f start_point(instance.start_loc.x,
          instance.start_loc.y);
      start_point = topological_mapper::toGrid(start_point, map_info_);

      current_graph_idx_ = 
        topological_mapper::getClosestIdOnGraph(start_point, graph_);
      float direction = instance.start_loc.yaw;

      ROS_INFO_STREAM("Start: " << current_graph_idx_ << 
          ", direction: " << direction << 
          ", num_robots: " << num_robots_available_ - assigned_robots_);

      checkRobotPlacementAtCurrentState(direction);

    }

    virtual void checkRobotPlacementAtCurrentState(float direction) {

      boost::mutex::scoped_lock lock(robot_modification_mutex_);

      if (assigned_robots_ >= num_robots_available_ ||
          current_graph_idx_ == goal_idx_) {
        // Already placed all available robots or no need to
        return;
      }

      const Instance& instance = 
        getInstance(experiment_, instance_name_);

      // Given the current graph id of the person and the direction the person
      // is moving in, compute the expected forward locations of the person
      std::vector<size_t> states;
      size_t current_id = current_graph_idx_;
      float current_direction = direction;

      while(true) {

        states.push_back(current_id);

        // Compute all adjacent vertices from this location
        Graph::vertex_descriptor vd = boost::vertex(current_id, graph_);
        topological_mapper::Point2f loc = graph_[vd].location;
        std::vector<size_t> adjacent_vertices;
        topological_mapper::getAdjacentVertices(
            current_id, graph_, adjacent_vertices);

        // Check vertex that has most likely transition
        size_t next_vertex = (size_t)-1;
        float next_vertex_closeness = M_PI / 4;
        float next_angle = 0;
        for (std::vector<size_t>::const_iterator av = adjacent_vertices.begin();
            av != adjacent_vertices.end(); ++av) {
          Graph::vertex_descriptor next_vd = boost::vertex(*av, graph_);
          topological_mapper::Point2f next_loc = graph_[next_vd].location;
          float angle = atan2f((next_loc-loc).y, (next_loc-loc).x);
          // Wrap angle around direction
          while (angle <= current_direction - M_PI) angle += 2 * M_PI;
          while (angle > current_direction + M_PI) angle -= 2 * M_PI;
          // Take difference
          float closeness = fabs(angle - current_direction);
          if (closeness < next_vertex_closeness) {
            next_vertex = *av;
            next_vertex_closeness = closeness;
            next_angle = angle; 
          }
        }

        if (next_vertex == (size_t)-1) {
          // No close vertices found, break
          break;
        }

        // get normalize direction and id
        current_direction = atan2f(sinf(next_angle), cosf(next_angle)); 
        current_id = next_vertex;
      }

      // Now for each state in the forward path, see which is the closest
      size_t min_graph_idx = (size_t) -1;
      size_t point_to_vertex = (size_t) - 1;
      float min_distance = std::numeric_limits<float>::max();
      for (std::vector<size_t>::iterator si = states.begin(); 
          si != states.end(); ++si) {
        std::vector<size_t> path_from_goal;
        topological_mapper::getShortestPath(
            graph_, goal_idx_, *si, path_from_goal);
        path_from_goal.insert(path_from_goal.begin(), goal_idx_);
        float distance = 0;
        for (size_t pp = 0; pp < path_from_goal.size() - 1; ++pp) {
          Graph::vertex_descriptor vd1 = 
            boost::vertex(path_from_goal[pp], graph_);
          Graph::vertex_descriptor vd2 = 
            boost::vertex(path_from_goal[pp + 1], graph_);
          distance += topological_mapper::getMagnitude(
              graph_[vd1].location - graph_[vd2].location);
        }
        if (distance < min_distance) {
          min_graph_idx = *si;
          point_to_vertex = path_from_goal[1];
          min_distance = distance;
        }
      }

      // If the minimum idx is the current location, place a robot here
      if (min_graph_idx == current_graph_idx_) {

        topological_mapper::Point2f at_loc = 
          topological_mapper::getLocationFromGraphId(
              current_graph_idx_, graph_);

        topological_mapper::Point2f from_loc =
          at_loc - topological_mapper::Point2f(
              50.0 * cosf(direction),
              50.0 * sinf(direction));

        topological_mapper::Point2f to_loc = 
          topological_mapper::getLocationFromGraphId(
              point_to_vertex, graph_);

        // Compute robot pose
        geometry_msgs::Pose pose = positionRobot(from_loc, at_loc, to_loc);
        float robot_yaw = tf::getYaw(pose.orientation);

        // Compute arrow pose
        topological_mapper::Point2f robot_loc(pose.position.x, pose.position.y);
        robot_loc = topological_mapper::toGrid(robot_loc, map_info_);
        topological_mapper::Point2f change_loc = to_loc - robot_loc;
        float destination_yaw = atan2(change_loc.y, change_loc.x);
        float change_in_yaw = destination_yaw - robot_yaw;
        cv::Mat robot_image;
        produceDirectedArrow(change_in_yaw, robot_image);

        // Teleport the robot and assign a direction
        std::string robot_id = default_robots_.robots[assigned_robots_].id;
        robot_locations_[robot_id] = pose;
        robot_screen_orientations_[robot_id] = destination_yaw;
        robot_screen_publisher_.updateImage(robot_id, robot_image);
        ++assigned_robots_;
      }

    }

    virtual void odometryCallback(const nav_msgs::Odometry::ConstPtr odom) {
      
      if (!instance_in_progress_)
        return;

      topological_mapper::Point2f person_loc(
          odom->pose.pose.position.x,
          odom->pose.pose.position.y);
      person_loc = topological_mapper::toGrid(person_loc, map_info_);

      size_t next_graph_id =
        topological_mapper::getClosestIdOnGraphFromEdge(person_loc,
            graph_, current_graph_idx_);

      if (next_graph_id != current_graph_idx_) {
        // A transition has happened. Compute next state and check if robot 
        // needs to be placed.
        Graph::vertex_descriptor vd1 = 
          boost::vertex(current_graph_idx_, graph_);
        topological_mapper::Point2f loc1 = graph_[vd1].location;
        Graph::vertex_descriptor vd2 = boost::vertex(next_graph_id, graph_);
        topological_mapper::Point2f loc2 = graph_[vd2].location;
        float direction = atan2f((loc2-loc1).y, (loc2-loc1).x);
        current_graph_idx_ = next_graph_id;
        checkRobotPlacementAtCurrentState(direction);
        ROS_INFO_STREAM("Transitioning to: " << current_graph_idx_ << 
            ", direction: " << direction << 
            ", num_robots: " << num_robots_available_ - assigned_robots_);
      }
    }

};


int main(int argc, char** argv) {

  ros::init(argc, argv, "heuristic_robot_positioner");
  boost::shared_ptr<ros::NodeHandle> nh;
  nh.reset(new ros::NodeHandle());
  HeuristicRobotPositioner rp(nh);
  rp.run();

  return 0;
}
