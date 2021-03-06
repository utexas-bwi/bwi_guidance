#include <fstream>
#include <pluginlib/class_list_macros.h>

#include <bwi_guidance_solver/irm/vi_solver.h>

namespace bwi_guidance_solver {

  namespace irm {

    bool VISolver::initializeSolverSpecific(Json::Value &params) {
      vi_params_.fromJson(params);
      vi_params_.max_value = 0.0f;
      vi_params_.min_value = -10000.0f;
      if (!boost::filesystem::is_directory(base_directory_ + "/vi") &&
          !boost::filesystem::create_directory(base_directory_ + "/vi"))
      {
        return false;
      }
      return true;
    }

    void VISolver::resetSolverSpecific() {
      estimator_.reset(new PersonEstimator);

      vi_.reset(new ValueIteration<State, Action>(model_, estimator_, vi_params_));

      std::ostringstream vi_policy_file_ss;
      vi_policy_file_ss << std::fixed << std::setprecision(2);
      vi_policy_file_ss << base_directory_ << "/vi/" << "goal" << goal_idx_ << "_gamma" << vi_params_.gamma <<
        "_policy.txt";

      std::string vi_policy_file = vi_policy_file_ss.str();

      std::ifstream vi_ifs(vi_policy_file.c_str());
      if (vi_ifs.good()) {
        std::cout << "VI policy file found: " << vi_policy_file << std::endl;
        vi_->loadPolicy(vi_policy_file);
      } else {
        std::cout << "VI policy file not found. Computing policy... " << std::flush;
        vi_->computePolicy();
        vi_->savePolicy(vi_policy_file);
        std::cout << "DONE!" << std::endl;
      }
      vi_ifs.close();
    }

    Action VISolver::getBestAction(const State &state) {
      return vi_->getBestAction(state);
    }

    std::string VISolver::getSolverName() {
      return std::string("VI");
    }

    void VISolver::precomputeAndSavePolicy(int problem_identifier) {
      float pixel_visibility_range = domain_params_.visibility_range / map_.info.resolution;
      std::string empty_model_file;
      model_.reset(new PersonModel(graph_, map_, problem_identifier, empty_model_file,
            domain_params_.allow_robot_current_idx, pixel_visibility_range, domain_params_.allow_goal_visibility));
      goal_idx_ = problem_identifier;
      seed_ = problem_identifier;

      // Generating the VI instances will automatically compute the policy if it has not been computed.
      this->resetSolverSpecific();
    }

    std::map<std::string, std::string> VISolver::getParamsAsMapSolverSpecific() {
      return vi_params_.asMap();
    }

  }

}

PLUGINLIB_EXPORT_CLASS(bwi_guidance_solver::irm::VISolver, bwi_guidance_solver::irm::Solver)
