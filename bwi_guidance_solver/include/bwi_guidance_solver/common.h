#ifndef BWI_GUIDANCE_SOLVER_COMMON_H
#define BWI_GUIDANCE_SOLVER_COMMON_H

namespace bwi_guidance {

  const unsigned NUM_DIRECTIONS = 16;

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
      std::map<int, std::vector<int> > adjacent_vertices_map,
      const bwi_mapper::Graph& graph);

  void computeVisibleVertices(
      std::map<int, std::vector<int> > adjacent_vertices_map,
      const bwi_mapper::Graph& graph,
      float visibility_range);
};

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_COMMON_H */
