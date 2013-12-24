#ifndef HEURISTIC_SOLVER_CBV4SH6M
#define HEURISTIC_SOLVER_CBV4SH6M

#include <bwi_exp1_solver/structures.h>
#include <nav_msgs/OccupancyGrid.h>
#include <bwi_mapper/graph.h>

class HeuristicSolver {

  public:
    HeuristicSolver(const nav_msgs::OccupancyGrid& map, 
        const bwi_mapper::Graph& graph, int goal_idx, 
        bool allow_robot_current_idx = false, float visibility_range = 0.0f,
        bool allow_goal_visibility_ = true);
    ~HeuristicSolver();
    void computePolicy();
    void loadPolicy(const std::string& file);
    void savePolicy(const std::string& file);
    bwi_exp1::Action getBestAction(const bwi_exp1::State& state) const;
    virtual std::string generateDescription(unsigned int indentation = 0) {
      return std::string("stub");
    }

  private:
    nav_msgs::OccupancyGrid map_;
    bwi_mapper::Graph graph_;
    int goal_idx_;
    bool allow_robot_current_idx_;
    float visibility_range_;
    bool allow_goal_visibility_;
    std::map<int, std::vector<int> > visible_vertices_map_;
};

#endif /* end of include guard: HEURISTIC_SOLVER_CBV4SH6M */
