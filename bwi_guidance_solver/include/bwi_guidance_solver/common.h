#ifndef BWI_GUIDANCE_SOLVER_COMMON_H
#define BWI_GUIDANCE_SOLVER_COMMON_H

#include <bwi_mapper/graph.h>

namespace bwi_guidance_solver {

  const unsigned NUM_DIRECTIONS = 16;

  struct LineToDraw {
    bool dashed;
    int priority;
    float precision;
    cv::Scalar color;
  };

  struct SquareToDraw {
    cv::Scalar color;
  };

  enum MDPConstants {
    NONE = -1,
    DIR_UNASSIGNED = -2
  };

  /* Helper Functions */
  int computeNextDirection(int dir, int graph_id, int next_graph_id, 
      const bwi_mapper::Graph& graph);
  int getDiscretizedAngle(float angle);
  float getAngleInRadians(int dir);
  float getAbsoluteAngleDifference(float angle1, float angle2);

  void computeAdjacentVertices(
      std::map<int, std::vector<int> >& adjacent_vertices_map,
      const bwi_mapper::Graph& graph);

  void computeVisibleVertices(
      std::map<int, std::vector<int> >& visible_vertices_map,
      const bwi_mapper::Graph& graph,
      const nav_msgs::OccupancyGrid& map,
      float visibility_range);

  void dashedLine(cv::Mat& image, cv::Point start, cv::Point goal,
      cv::Scalar color=cv::Scalar(0,0,0), int dash_width = 10, 
      int thickness=1, int linetype=4);
};

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_COMMON_H */
