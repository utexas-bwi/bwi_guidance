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

  void computeShorestPath(std::vector<std::vector<float> > &shortest_distances,
                          std::vector<std::vector<std::vector<size_t> > > &shortest_paths,
                          const bwi_mapper::Graph& graph) {

    int num_vertices = boost::num_vertices(graph);
    shortest_paths.resize(num_vertices);
    shortest_distances.resize(num_vertices);

    for (int idx = 0; idx < num_vertices; ++idx) {
      shortest_distances[idx].resize(num_vertices);
      shortest_paths[idx].resize(num_vertices);
      for (int j = 0; j < num_vertices; ++j) {
        if (j == idx) {
          shortest_distances[idx][j] = 0;
          shortest_paths[idx][j].clear();
        } else {
          shortest_distances[idx][j] = 
            bwi_mapper::getShortestPathWithDistance(idx, j, shortest_paths[idx][j], graph);
          // Post-process the shortest path - add goal, remove start and reverse
          shortest_paths[idx][j].insert(shortest_paths[idx][j].begin(), j); // Add j
          shortest_paths[idx][j].pop_back(); // Remove idx
          std::reverse(shortest_paths[idx][j].begin(), shortest_paths[idx][j].end());
        }
      }
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
    cv::Point2f head_center = loc; head_center.y -= 12;
    cv::circle(image, head_center, 7, cv::Scalar(192, 226, 255), -1, CV_AA); 
    cv::circle(image, head_center, 7, cv::Scalar(0, 0, 0), 1, CV_AA); 

    // Draw torso
    cv::Point torso_loc = loc;
    cv::Rect rect(torso_loc.x - 5, torso_loc.y - 5, 11, 12);
    cv::rectangle(image, rect, cv::Scalar(255, 137, 7), -1, CV_AA); 
    cv::rectangle(image, rect, cv::Scalar(0, 0, 0), 1, CV_AA); 

    // Draw legs
    cv::Point legs_loc = loc; legs_loc.y += 12;
    rect = cv::Rect(legs_loc.x - 5, legs_loc.y - 5, 11, 12);
    cv::rectangle(image, rect, cv::Scalar(138, 138, 138), -1, CV_AA); 
    cv::rectangle(image, rect, cv::Scalar(0, 0, 0), 1, CV_AA); 
    cv::line(image, legs_loc+cv::Point(0, -4), legs_loc+cv::Point(0, 6), cv::Scalar(0, 0, 0), 1, CV_AA);

  }

  void drawRobotOnImage(cv::Mat &image, const cv::Point2f &loc, const cv::Scalar &color) {

    // Draw head
    cv::Point head_center = loc; head_center.y -= 7;
    cv::line(image, head_center - cv::Point(0, 12), head_center - cv::Point(0, 6), cv::Scalar(0, 0, 0), 1, CV_AA); 
    cv::circle(image, head_center - cv::Point(0, 12), 3, cv::Scalar(0, 0, 0), -1, CV_AA); 
    cv::circle(image, head_center, 7, color, -1, CV_AA); 
    cv::circle(image, head_center, 7, cv::Scalar(0,0,0), 1, CV_AA); 

    // Draw torso
    cv::Point torso_loc = loc;
    cv::Rect rect(torso_loc.x - 7, torso_loc.y - 7, 15, 14);
    cv::rectangle(image, rect, color, -1, CV_AA); 
    cv::rectangle(image, rect, cv::Scalar(0, 0, 0), 1, CV_AA); 

    // Draw legs
    cv::Point legs_loc = loc; legs_loc.y += 7;
    cv::line(image, legs_loc+cv::Point(-4, 0), legs_loc+cv::Point(-4, 5), cv::Scalar(0, 0, 0), 2, CV_AA);
    cv::line(image, legs_loc+cv::Point(4, 0), legs_loc+cv::Point(4, 5), cv::Scalar(0, 0, 0), 2, CV_AA);

  }

  void drawCheckeredFlagOnImage(cv::Mat &image, const cv::Point2f &loc) {

    int num_rows = 4;
    int num_cols = 6;
    int square_size = 5;

    for (int row = 0; row < num_rows; ++row) {
      bool draw_black = (row % 2) == 0;
      for (int col = 0; col < num_cols; ++col) {
        cv::Rect rect(loc.x + square_size * (col - num_cols/2), loc.y + square_size * (row - num_rows/2),
                      square_size, square_size);
        cv::Scalar color = (draw_black) ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255);
        cv::rectangle(image, rect, color, -1, CV_AA); 
        draw_black = !draw_black;
      }
    }
    cv::Rect rect(loc.x + square_size * (-num_cols/2) - 1, loc.y + square_size * (-num_rows / 2) - 1, 
                  num_cols * square_size + 2, num_rows * square_size + 2);
    cv::rectangle(image, rect, cv::Scalar(0, 0, 0), 1, CV_AA); 

  }

  void drawScreenWithDirectedArrowOnImage(cv::Mat &image, const cv::Point2f &robot_loc, float orientation) {

    int screen_size_x = 80;

    int screen_size_y = (screen_size_x * 3) / 4;
    cv::Rect rect(robot_loc.x + screen_size_x / 6, robot_loc.y - screen_size_y / 2, screen_size_x, screen_size_y);
    cv::rectangle(image, rect, cv::Scalar(255, 255, 255), -1, CV_AA); 
    cv::rectangle(image, rect, cv::Scalar(0, 0, 0), 2, CV_AA); 

    bwi_mapper::drawArrowOnImage(image, robot_loc + cv::Point2f((2 * screen_size_x) / 3, 0), orientation + M_PI / 2, 
                                 cv::Scalar(0, 0, 0), screen_size_x / 4, screen_size_x / 40);
  }

  void drawScreenWithFollowMeText(cv::Mat &image, const cv::Point2f &robot_loc) {

    int screen_size_x = 80;

    int screen_size_y = (screen_size_x * 3) / 4;
    cv::Rect rect(robot_loc.x + screen_size_x / 6, robot_loc.y - screen_size_y / 2, screen_size_x, screen_size_y);
    cv::rectangle(image, rect, cv::Scalar(255, 255, 255), -1, CV_AA); 
    cv::rectangle(image, rect, cv::Scalar(0, 0, 0), 2, CV_AA); 

    cv::putText(image, "FOLLOW", robot_loc + cv::Point2f(20, -4), CV_FONT_HERSHEY_SIMPLEX, 0.53, cv::Scalar(0, 0, 0), 2);
    cv::putText(image, "ME", robot_loc + cv::Point2f(40, 12), CV_FONT_HERSHEY_SIMPLEX, 0.53, cv::Scalar(0, 0, 0), 2);
  }

} /* bwi_guidance */
