#include <bwi_guidance_solver/common.h>
#include <bwi_mapper/graph.h>

namespace bwi_guidance_solver {
  
  int computeNextDirection(int dir, int graph_id, 
      int next_graph_id, const bwi_mapper::Graph& graph) {
    float angle = 
      bwi_mapper::getNodeAngle(graph_id, next_graph_id, graph);
    return getDiscretizedAngle(angle);
  }

  int getDiscretizedAngle(float angle) {
    angle = angle + M_PI / NUM_DIRECTIONS;
    while (angle < 0) angle += 2 * M_PI;
    while (angle >= 2 * M_PI) angle -= 2 * M_PI;
    return (angle * NUM_DIRECTIONS) / (2 * M_PI);
  }

  float getAngleInRadians(int dir) {
    return ((2 * M_PI) / NUM_DIRECTIONS) * dir;
  }

  float getAbsoluteAngleDifference(float angle1, float angle2) {
    while (angle2 > angle1 + M_PI) angle2 -= 2 * M_PI;
    while (angle2 <= angle1 - M_PI) angle2 += 2 * M_PI;
    return fabs (angle1 - angle2);
  }

  void computeAdjacentVertices(
      std::map<int, std::vector<int> >& adjacent_vertices_map,
      const bwi_mapper::Graph& graph) {
    adjacent_vertices_map.clear();
    for (int graph_id = 0; graph_id < boost::num_vertices(graph); ++graph_id) {
      std::vector<size_t> adjacent_vertices;
      bwi_mapper::getAdjacentNodes(graph_id, graph, adjacent_vertices); 
      adjacent_vertices_map[graph_id] = 
        std::vector<int>(adjacent_vertices.begin(), adjacent_vertices.end());
    }
  }

  void computeVisibleVertices(
      std::map<int, std::vector<int> >& visible_vertices_map,
      const bwi_mapper::Graph& graph,
      const nav_msgs::OccupancyGrid& map,
      float visibility_range) {
    visible_vertices_map.clear();
    for (int graph_id = 0; graph_id < boost::num_vertices(graph); ++graph_id) {
      std::vector<size_t> visible_vertices;
      bwi_mapper::getVisibleNodes(graph_id, graph, map,
          visible_vertices, visibility_range); 
      visible_vertices_map[graph_id] = 
        std::vector<int>(visible_vertices.begin(), visible_vertices.end());
    }
  }

  void dashedLine(cv::Mat& image, cv::Point start, cv::Point goal,
      cv::Scalar color, int dash_width, int thickness, int linetype) {
    cv::LineIterator it(image, start, goal, 8);   
    for (int i = 0; i < it.count; i+=2*dash_width) {
      if (i >= it.count - 2*dash_width) {
        cv::line(image, it.pos(), goal, color, thickness, linetype); 
        break;
      }
      cv::Point p1 = it.pos(); 
      for (int j = 0; j < dash_width; ++j) it++;
      cv::Point p2 = it.pos(); 
      for (int j = 0; j < dash_width; ++j) it++;
      cv::line(image, p1, p2, color, thickness, linetype);
    }
  }

  void drawPersonOnImage(cv::Mat &image, const cv::Point2f &loc) {

    // Draw head
    cv::Point2f head_center = loc; head_center.y -= 10;
    cv::circle(image, head_center, 6, cv::Scalar(192, 226, 255), -1, CV_AA); 
    cv::circle(image, head_center, 6, cv::Scalar(0, 0, 0), 1, CV_AA); 

    // Draw torso
    cv::Point torso_loc = loc;
    cv::Rect rect(torso_loc.x - 4, torso_loc.y - 4, 9, 10);
    cv::rectangle(image, rect, cv::Scalar(255, 137, 7), -1, CV_AA); 
    cv::rectangle(image, rect, cv::Scalar(0, 0, 0), 1, CV_AA); 

    // Draw legs
    cv::Point legs_loc = loc; legs_loc.y += 10;
    rect = cv::Rect(legs_loc.x - 4, legs_loc.y - 5, 9, 10);
    cv::rectangle(image, rect, cv::Scalar(138, 138, 138), -1, CV_AA); 
    cv::rectangle(image, rect, cv::Scalar(0, 0, 0), 1, CV_AA); 
    cv::line(image, legs_loc+cv::Point(0, -5), legs_loc+cv::Point(0, 5), cv::Scalar(0, 0, 0), 1, CV_AA);

  }

  void drawRobotOnImage(cv::Mat &image, const cv::Point2f &loc, const cv::Scalar &color) {

    // Draw head
    cv::Point head_center = loc; head_center.y -= 6;
    cv::line(image, head_center - cv::Point(0, 9), head_center - cv::Point(0, 5), cv::Scalar(0, 0, 0), 1, CV_AA); 
    cv::circle(image, head_center - cv::Point(0, 9), 2, cv::Scalar(0, 0, 0), -1, CV_AA); 
    cv::circle(image, head_center, 6, color, -1, CV_AA); 
    cv::circle(image, head_center, 6, cv::Scalar(0,0,0), 1, CV_AA); 

    // Draw torso
    cv::Point torso_loc = loc;
    cv::Rect rect(torso_loc.x - 6, torso_loc.y - 6, 13, 12);
    cv::rectangle(image, rect, color, -1, CV_AA); 
    cv::rectangle(image, rect, cv::Scalar(0, 0, 0), 1, CV_AA); 

    // Draw legs
    cv::Point legs_loc = loc; legs_loc.y += 8;
    cv::line(image, legs_loc+cv::Point(-3, -2), legs_loc+cv::Point(-3, 3), cv::Scalar(0, 0, 0), 1, CV_AA);
    cv::line(image, legs_loc+cv::Point(3, -2), legs_loc+cv::Point(3, 3), cv::Scalar(0, 0, 0), 1, CV_AA);

  }

  void drawCheckeredFlagOnImage(cv::Mat &image, const cv::Point2f &loc) {

    for (int row = 0; row < 3; ++row) {
      bool draw_black = (row % 2) == 0;
      for (int col = 0; col < 5; ++col) {
        cv::Rect rect(loc.x + 4 * (col - 2), loc.y + 4 * (row - 2) + 2, 4, 4);
        cv::Scalar color = (draw_black) ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255);
        cv::rectangle(image, rect, color, -1, CV_AA); 
        draw_black = !draw_black;
      }
    }
    cv::Rect rect(loc.x - 9, loc.y - 7, 22, 14);
    cv::rectangle(image, rect, cv::Scalar(0, 0, 0), 1, CV_AA); 

  }

  void drawScreenWithDirectedArrowOnImage(cv::Mat &image, const cv::Point2f &robot_loc, float orientation) {
    cv::Rect rect(robot_loc.x + 10, robot_loc.y - 22, 65, 45);
    cv::rectangle(image, rect, cv::Scalar(255, 255, 255), -1, CV_AA); 
    cv::rectangle(image, rect, cv::Scalar(0, 0, 0), 2, CV_AA); 
    bwi_mapper::drawArrowOnImage(image, robot_loc + cv::Point2f(40, 0), orientation + M_PI / 2, 
                                 cv::Scalar(0, 0, 0), 17, 2);
  }

  void drawScreenWithFollowMeText(cv::Mat &image, const cv::Point2f &robot_loc) {
    cv::Rect rect(robot_loc.x + 10, robot_loc.y - 22, 65, 45);
    cv::rectangle(image, rect, cv::Scalar(255, 255, 255), -1, CV_AA); 
    cv::rectangle(image, rect, cv::Scalar(0, 0, 0), 2, CV_AA); 
    cv::putText(image, "FOLLOW", robot_loc + cv::Point2f(15, -2), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    cv::putText(image, "ME", robot_loc + cv::Point2f(30, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
  }

} /* bwi_guidance */
