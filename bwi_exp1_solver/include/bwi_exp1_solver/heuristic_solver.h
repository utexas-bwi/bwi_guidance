#ifndef HEURISTIC_SOLVER_CBV4SH6M
#define HEURISTIC_SOLVER_CBV4SH6M

#include <bwi_exp1_solver/structures.h>
#include <nav_msgs/OccupancyGrid.h>
#include <topological_mapper/graph.h>

class HeuristicSolver {

  public:
    HeuristicSolver(const nav_msgs::OccupancyGrid& map, 
        const topological_mapper::Graph& graph, int goal_idx, 
        bool allow_robot_current_idx = false);
    ~HeuristicSolver();
    void computePolicy();
    void loadPolicy(const std::string& file);
    void savePolicy(const std::string& file);
    bwi_exp1::Action getBestAction(const bwi_exp1::State2& state) const;
    virtual std::string generateDescription(unsigned int indentation = 0) {
      return std::string("stub");
    }

  private:
    nav_msgs::OccupancyGrid map_;
    topological_mapper::Graph graph_;
    int goal_idx_;
    bool allow_robot_current_idx_;
};

#endif /* end of include guard: HEURISTIC_SOLVER_CBV4SH6M */
