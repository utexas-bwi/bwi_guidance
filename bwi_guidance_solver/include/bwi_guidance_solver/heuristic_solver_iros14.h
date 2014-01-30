#ifndef HEURISTIC_SOLVER_IROS14_H
#define HEURISTIC_SOLVER_IROS14_H

#include <bwi_guidance_solver/heuristic_solver_qrr14.h>
#include <bwi_guidance_solver/structures_iros14.h>
#include <bwi_guidance_solver/person_model_iros14.h>

class HeuristicSolverIROS14 : public HeuristicSolver {

  public:
    HeuristicSolverIROS14(const nav_msgs::OccupancyGrid& map, 
        const bwi_mapper::Graph& graph, int goal_idx, bool improved = false,
        float human_speed = 1.0f);
    ~HeuristicSolverIROS14();
    void computePolicy();
    void loadPolicy(const std::string& file);
    void savePolicy(const std::string& file);
    bwi_guidance::ActionIROS14 getBestAction(
        const bwi_guidance::StateIROS14& state,
        const boost::shared_ptr<bwi_guidance::PersonModelIROS14>& evaluation_model);
    virtual std::string generateDescription(unsigned int indentation = 0) {
      return std::string("stub");
    }

  protected:
    bool improved_;
    float human_speed_;
};

#endif /* end of include guard: HEURISTIC_SOLVER_IROS14_H */
