#include <bwi_guidance_solver/instantanuous/domain.h>
#include <bwi_guidance_solver/instantanuous/solver.h>

#include <pluginlib/class_loader.h>

namespace bwi_guidance_solver {

  namespace irm {

    struct InstanceResult {
      std::vector<MethodResult> results;
      std::vector<MethodResult> normalized_results;
    };

    bool Domain::initialize(Json::Value &experiment, const std::string &base_directory) {

      // Read any domain-level parameters.
      params_.fromJson(experiment["params"]);

      // Read the map and graph file.
      if (boost::filesystem::is_regular_file(params_.map_file) &&
          boost::filesystem::is_regular_file(params_.graph_file)) {
        // Read map and graph
        bwi_mapper::MapLoader mapper(map_file_);
        mapper.getMap(map_);
        bwi_mapper::readGraphFromFile(graph_file_, map.info, graph);
      } else {
        // Print an error should the map file or graph file not exist.
        ROS_FATAL_STREAM("Either the map file (" << params_.map_file << ") or the graph file (" << 
            params_.graph_file << ") does not exist.");
        return false;
      }

      // Compute the base directory
      std::ostringstream parametrized_dir_ss;
      parametrized_dir_ss << std::fixed << std::setprecision(2);
      parametrized_dir_ss << base_directory << "/irm" << 
        "-dl" << params_.distance_limit << 
        "-arci" << params_.allow_robot_current_idx <<
        "-vr" << params_.visibility_range <<
        "-agv" << params_.allow_goal_visibility;
      base_directory_ = parametrized_dir_ss.str();
      if (!boost::filesystem::create_directory(base_directory_))
      {
        ROS_FATAL_STREAM("Could not create the following domain directory: " << base_directory_);
        return false;
      }

      // Load all the solvers we'll be testing 
      Json::Value solvers = experiment["solvers"];
      pluginlib::ClassLoader<Solver> class_loader("bwi_guidance_solver",
          "bwi_guidance_solver::irm::Solver");
      try {
        for (unsigned solver_idx = 0; solver_idx < solvers.size(); ++solver_idx) {
          std::string solver_name = solvers[solver_idx]["name"].asString();
          boost::shared_ptr<Solver> solver = class_loader.createInstance(solver_name);
          if (!(solver->initialize(solvers[solver_idx]["params"], map_, graph_, base_directory_))) {
            ROS_FATAL("Could not initialize solver.");
            return false;
          }
          solvers_.push_back(solver);
        }
      } catch(pluginlib::PluginlibException& ex) {
        // Print an error should any solver fail to load.
        ROS_FATAL("Unable to load an irm Solver. Error: %s", ex.what());
        return false;
      }

      return true;
    }

    void Domain::precomputeAndSavePolicy(int problem_identifier) {
      BOOST_FOREACH(boost::shared_ptr<Solver>& solver, solvers_) {
        solver->precomputeAndSavePolicy(problem_identifier);
      }
    }

    void Domain::testInstance(int seed) {

      boost::mt19937 mt(seed_ + i);
      boost::uniform_int<int> idx_dist(0, boost::num_vertices(graph) - 1);
      UIGen idx_gen(mt, idx_dist);
      boost::uniform_int<int> direction_dist(0, 15);
      UIGen direction_gen(mt, direction_dist);

      int start_idx = idx_gen();
      int goal_idx = idx_gen();
      while (goal_idx == start_idx) {
        goal_idx = idx_gen();
      }
      int start_direction = direction_gen();
      
      float pixel_visibility_range = params_.visibility_range / map_.info.resolution;

      std::string empty_model_file; // Do not load or save models to file
      float pixel_visibility_range = visibility_range_ / map.info.resolution;

      // We can create the model right now as loading the model is not dependent on the method parameters. We just need
      // to make sure we reinitialize the rng and update the reward structure for every method as necessary
      boost::shared_ptr<PersonModel> model(new PersonModel(graph_, map_, goal_idx, empty_model_file,
            params_.allow_robot_current_idx_, params_.pixel_visibility_range, params_.allow_goal_visibility));

      BOOST_FOREACH(boost::shared_ptr<Solver>& solver, solvers_) {

    // MethodResult method_result;

    // boost::shared_ptr<HeuristicSolver> hs;
    // boost::shared_ptr<ValueIteration<State, Action> > vi;
    // boost::shared_ptr<MCTS<State, Action> > mcts;
        model->updateRewardStructure(solver_->success_reward, solver_->reward_structure);

        solver->reset(seed, goal_idx);

    // if (params.type == HEURISTIC) {
    //   hs.reset(new HeuristicSolver(map, graph, goal_idx,
    //         allow_robot_current_idx_, pixel_visibility_range,
    //         allow_goal_visibility_)); 
    // } else if (params.type == VI) {
    //   //boost::shared_ptr<PersonEstimator> estimator;
    //   estimator.reset(new PersonEstimator);
    //   vi = getVIInstance(map, model, estimator, goal_idx, params);
    // } else if (params.type == MCTS_TYPE) {

    //   if (!mcts_enabled_) {
    //     throw std::runtime_error(
    //         std::string("MCTS method present, but no global MCTS ") +
    //         "parameter file provided. Please set the mcts-params flag.");
    //   }

    //   UCTEstimator<State, Action>::Params uct_estimator_params;
    //   uct_estimator_params.gamma = params.gamma;
    //   uct_estimator_params.lambda = params.lambda;
    //   uct_estimator_params.rewardBound = params.mcts_reward_bound;
    //   uct_estimator_params.useImportanceSampling =
    //     params.mcts_importance_sampling;

    //   // Create the RNG required for mcts rollouts
    //   boost::shared_ptr<RNG> mcts_rng(new RNG(1 * (seed + 1)));

    //   boost::shared_ptr<ModelUpdaterSingle<State, Action> >
    //     mcts_model_updator(
    //         new ModelUpdaterSingle<State, Action>(model));
    //   boost::shared_ptr<IdentityStateMapping<State> > mcts_state_mapping(
    //       new IdentityStateMapping<State>);
    //   boost::shared_ptr<UCTEstimator<State, Action> > uct_estimator(
    //       new UCTEstimator<State, Action>(mcts_rng,
    //         uct_estimator_params));
    //   mcts.reset(new MCTS<State, Action>(uct_estimator,
    //         mcts_model_updator, mcts_state_mapping, mcts_rng, mcts_params_));
    // }

        boost::mt19937 mt(2 * (seed + 1));
        boost::uniform_real<float> u(0.0f, 1.0f);
        URGenPtr transition_rng(new URGen(mt, u));

        for (int starting_robots = 1; starting_robots <= MAX_ROBOTS;
            ++starting_robots) {

          EVALUATE_OUTPUT("Evaluating method " << params << " with " << 
              starting_robots << " robots.");

          State current_state; 
          current_state.graph_id = start_idx;
          current_state.direction = start_direction;
          current_state.num_robots_left = starting_robots;
          current_state.robot_direction = NONE;
          current_state.visible_robot = NONE;

          float reward = 0;
          float instance_distance = 0;

          EVALUATE_OUTPUT(" - start " << current_state);

          solver->performEpisodeStartComputation();
      // method_result.mcts_terminations[starting_robots - 1] = 0;
      // method_result.mcts_playouts[starting_robots - 1] = 0;
      // if (params.type == MCTS_TYPE) {
      //   mcts->restart();
      //   EVALUATE_OUTPUT(" - performing initial MCTS search for " +
      //       boost::lexical_cast<std::string>(
      //         params.mcts_initial_planning_time) + "s");
      //   for (int i = 0; i < params.mcts_initial_planning_time; ++i) {
      //     unsigned int playouts, terminations;
      //     playouts = mcts->search(current_state, terminations);
      //     method_result.mcts_playouts[starting_robots - 1] = playouts;
      //     method_result.mcts_terminations[starting_robots - 1] += terminations;
      //   }
      // }

          float distance_limit_pxl = ((float)distance_limit_) / map.info.resolution;

          while (current_state.graph_id != goal_idx && instance_distance <= distance_limit_pxl) {

            std::vector<State> next_states;
            std::vector<float> probabilities;
            std::vector<float> rewards;

            // Deterministic system transitions
            while (true) {

              Action action = solver->getBestAction(current_state);
              EVALUATE_OUTPUT("   action: " << action);

              model->getTransitionDynamics(current_state, action, next_states, rewards, probabilities);

              if (action.type == DO_NOTHING) {
                // Manual transition
                break;
              }

              // The human does not move for this action, and a single next state
              // is present
              current_state = next_states[0];
              solver->performPostActionComputation();
              // if (params.type == MCTS_TYPE) {
              //   EVALUATE_OUTPUT(" - performing post-action MCTS search for 1s");
              //   unsigned int terminations;
              //   mcts->search(current_state, terminations);
              // }
              EVALUATE_OUTPUT(" - auto " << current_state);
            }

            // Select next state choice based on probabilities
            int choice = select(probabilities, transition_rng);
            State old_state = current_state;
            current_state = next_states[choice];
            float transition_distance = 
              bwi_mapper::getEuclideanDistance(old_state.graph_id, current_state.graph_id, graph);
            instance_distance += transition_distance;

            // Perform an MCTS search after next state is decided (not perfect)
            // Only perform search if system is left with any future action choice
            if (!model->isTerminalState(current_state))
            {
              distance = transition_distance * map_.info.resolution;
              solver->performPostActionComputation(distance)
              
              // &&
              //   (current_state.num_robots_left != 0 ||
              //    current_state.visible_robot != NONE)) {
              // if (params.type == MCTS_TYPE) {
              //   // Assumes 1m/s velocity for converting distance to time
              //   int distance = transition_distance * map.info.resolution;
              //   distance += params.mcts_planning_time_multiplier;
              //   EVALUATE_OUTPUT(" - performing post-wait MCTS search for " <<
              //       distance << "s");
              //   for (int i = 0; i < distance; ++i) {
              //     unsigned int terminations;
              //     mcts->search(current_state, terminations);
              //   }
              // }
            }

            EVALUATE_OUTPUT(" - manual " << current_state);
            reward += rewards[choice];
          }
          method_result.reward[starting_robots - 1] = reward;
          method_result.distance[starting_robots - 1] = instance_distance * map.info.resolution;
        }
        result.results.push_back(method_result);
  }

  // Produce normalized results - distance is easy
  float normalization_distance = getDistanceNormalizationValue(graph, goal_idx, start_idx) * map.info.resolution;

  // TODO add normalized distance to the results. 

  return result;
}
    }

  } /* irm - InstantaneousRobotMotion */

} /* bwi_guidance_solver */
