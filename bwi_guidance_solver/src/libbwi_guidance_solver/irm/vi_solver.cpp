#include <fstream>
#include <bwi_guidance_solver/irm/vi_solver.h>

namespace bwi_guidance_solver {

  namespace irm {

    bool VISolver::initializeSolverSpecific(Json::Value &params) {
      vi_params_.fromJson(params);
      vi_params_.max_value = 0.0f;
      vi_params_.min_value = -10000.0f;
      return true;
    }

    void VISolver::reset() {
      estimator_.reset(new PersonEstimator);

      vi_.reset(new ValueIteration<State, Action>(model_, estimator_, vi_params_));

      std::ostringstream vi_policy_file_ss;
      vi_policy_file_ss << std::fixed << std::setprecision(2);
      vi_policy_file_ss << base_directory_ << "/vi/" << "goal" << goal_idx_ << "_gamma" << vi_params_.gamma <<
        "_policy.txt";

      std::string vi_policy_file = vi_policy_file_ss.str();

      std::ifstream vi_ifs(vi_policy_file.c_str());
      if (vi_ifs.good()) {
        vi_->loadPolicy(vi_policy_file);
      } else {
        vi_->computePolicy();
        vi_->savePolicy(vi_policy_file);
      }
      vi_ifs.close();
    }

    Action VISolver :: getBestAction(const State &state) {
      return vi_->getBestAction(state);
    }

  }

}

