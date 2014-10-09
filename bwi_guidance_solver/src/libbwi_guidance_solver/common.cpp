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

} /* bwi_guidance */
