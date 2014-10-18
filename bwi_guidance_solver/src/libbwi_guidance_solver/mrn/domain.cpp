#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv/highgui.h>

#include <pluginlib/class_list_macros.h>

#include <bwi_mapper/map_loader.h>
#include <bwi_guidance_solver/mrn/domain.h>
#include <bwi_guidance_solver/mrn/person_model.h>
#include <bwi_guidance_solver/mrn/solver.h>
#include <bwi_guidance_solver/mrn/structures.h>
#include <bwi_tools/record_writer.h>
#include <bwi_tools/resource_resolver.h>

#ifdef EVALUATE_DEBUG
  #define EVALUATE_OUTPUT(x) std::cout << x << std::endl;
#else
  #define EVALUATE_OUTPUT(x)
#endif

namespace bwi_guidance_solver {

  namespace mrn {

    Domain::~Domain() {
      // Deallocate every solver before exiting.
      BOOST_FOREACH(boost::shared_ptr<Solver>& solver, solvers_) {
        solver.reset();
      }
    }

    bool Domain::initialize(Json::Value &experiment, const std::string &base_directory) {

      // Read any domain-level parameters.
      params_.fromJson(experiment["params"]);

      params_.map_file = bwi_tools::resolveRosResource(params_.map_file);
      params_.graph_file = bwi_tools::resolveRosResource(params_.graph_file);

      // Read the map and graph file.
      if (boost::filesystem::is_regular_file(params_.map_file) &&
          boost::filesystem::is_regular_file(params_.graph_file)) {
        // Read map and graph
        bwi_mapper::MapLoader mapper(params_.map_file);
        mapper.getMap(map_);
        mapper.drawMap(base_image_);
        bwi_mapper::readGraphFromFile(params_.graph_file, map_.info, graph_);
      } else {
        // Print an error should the map file or graph file not exist.
        ROS_FATAL_STREAM("Either the map file (" << params_.map_file << ") or the graph file (" << 
            params_.graph_file << ") does not exist.");
        return false;
      }

      // Compute the base directory
      std::ostringstream parametrized_dir_ss;
      parametrized_dir_ss << std::fixed << std::setprecision(2);
      parametrized_dir_ss << base_directory << "/mrn" << 
        "-dl" << params_.distance_limit << 
        "-hs" << params_.human_speed <<
        "-rs" << params_.robot_speed <<
        "-um" << params_.utility_multiplier <<
        "-usr" << params_.use_shaping_reward <<
        "-dba" << params_.discourage_bad_assignments;
      base_directory_ = parametrized_dir_ss.str();
      if (!boost::filesystem::is_directory(base_directory_) &&
          !boost::filesystem::create_directory(base_directory_))
      {
        ROS_FATAL_STREAM("Could not create the following domain directory: " << base_directory_);
        return false;
      }
      if (!boost::filesystem::is_directory(base_directory_ + "/results") && 
          !boost::filesystem::create_directory(base_directory_ + "/results"))
      {
        ROS_FATAL_STREAM("Could not create the following results directory: " << base_directory_ + "/results");
        return false;
      }

      // Load all the solvers we'll be testing 
      Json::Value solvers = experiment["solvers"];
      class_loader_.reset(new pluginlib::ClassLoader<Solver>("bwi_guidance_solver", "bwi_guidance_solver::mrn::Solver"));
      try {
        for (unsigned solver_idx = 0; solver_idx < solvers.size(); ++solver_idx) {
          std::string solver_name = solvers[solver_idx]["name"].asString();
          boost::shared_ptr<Solver> solver = class_loader_->createInstance(solver_name);
          if (!(solver->initialize(params_, solvers[solver_idx]["params"], map_, graph_, base_directory_))) {
            ROS_FATAL("Could not initialize solver.");
            return false;
          }
          solvers_.push_back(solver);
        }
      } catch(pluginlib::PluginlibException& ex) {
        // Print an error should any solver fail to load.
        ROS_FATAL("Unable to load a mrn Solver. Error: %s", ex.what());
        return false;
      }

      if (params_.frame_rate != 0.0f) {
        cv::namedWindow("out");
        cvStartWindowThread();
      }

      return true;
    }

    void Domain::precomputeAndSavePolicy(int problem_identifier) {
      BOOST_FOREACH(boost::shared_ptr<Solver>& solver, solvers_) {
        solver->precomputeAndSavePolicy(problem_identifier);
      }
    }

    void Domain::testInstance(int seed) {

      RNG rng(seed);
      int num_vertices = boost::num_vertices(graph_);
      int start_idx = (params_.start_colocated) ? ROBOT_HOME_BASE[rng.randomInt(9)] : rng.randomInt(num_vertices - 1);
      int goal_idx = rng.randomInt(num_vertices - 1);
      while (goal_idx == start_idx) {
        goal_idx = rng.randomInt(num_vertices - 1);
      }
      // TODO How this is not a #define somewhere? I couldn't locate it. Needs more searching.
      int start_direction = rng.randomInt(15);

      std::cout << "Testing instance with start_idx: " << start_idx << ", goal_idx: " << goal_idx << std::endl;
      
      // We can create the model right now as loading the model is not dependent on the method parameters. We just need
      // to make sure we reinitialize the rng and update the reward structure for every method as necessary
      int max_robots_in_use = 0;
      int action_vertex_visiblity_depth = 0;
      int action_vertex_adjacency_depth = 0;
      float visibility_range = 0.0f;
      BOOST_FOREACH(boost::shared_ptr<Solver>& solver, solvers_) {
        max_robots_in_use = std::max(max_robots_in_use, solver->getMaxRobotsInUse());
        action_vertex_visiblity_depth = 
          std::max(action_vertex_visiblity_depth, solver->getActionVertexVisibilityDepth());
        action_vertex_adjacency_depth = 
          std::max(action_vertex_adjacency_depth, solver->getActionVertexAdjacencyDepth());
        visibility_range = std::max(visibility_range, solver->getVisibilityRange());
      }
      boost::shared_ptr<PersonModel> evaluation_model(new PersonModel(graph_, 
                                                                      map_, 
                                                                      goal_idx, 
                                                                      params_.frame_rate,
                                                                      max_robots_in_use,
                                                                      action_vertex_visiblity_depth,
                                                                      action_vertex_adjacency_depth,
                                                                      visibility_range,
                                                                      params_.human_speed,
                                                                      params_.robot_speed,
                                                                      params_.utility_multiplier,
                                                                      params_.use_shaping_reward,
                                                                      params_.discourage_bad_assignments));

      std::vector<std::map<std::string, std::string> > records;
      BOOST_FOREACH(boost::shared_ptr<Solver>& solver, solvers_) {

        EVALUATE_OUTPUT("Evaluating solver " << solver->getSolverName());
        solver->reset(seed, goal_idx);
        std::map<std::string, std::string> record = solver->getParamsAsMap();

        record["name"] = solver->getSolverName();
        record["start_idx"] = boost::lexical_cast<std::string>(start_idx);
        record["goal_idx"] = boost::lexical_cast<std::string>(goal_idx);
        record["start_direction"] = boost::lexical_cast<std::string>(start_direction);

        boost::shared_ptr<RNG> evaluation_rng(new RNG(2 * (seed + 1)));

        State current_state; 
        current_state.graph_id = start_idx;
        current_state.direction = start_direction;
        current_state.precision = 1.0f;
        current_state.robot_gave_direction = false;
        evaluation_model->addRobots(current_state, params_.max_robots, evaluation_rng);

        if (params_.start_colocated) {
          // If the robot starts colocated as the human, then assign that robot to help the human.
          float unused_reward;
          int unused_depth_count;
          bool unused_terminal;
          State next_state;
          evaluation_model->takeAction(current_state, Action(ASSIGN_ROBOT, start_idx, DIR_UNASSIGNED), 
                                       unused_reward, next_state, unused_terminal, unused_depth_count, 
                                       evaluation_rng);
          current_state = next_state;
        }

        float instance_reward = 0.0f;
        float instance_distance = 0.0f;
        float instance_time = 0.0f;
        float instance_utility = 0.0f;

        EVALUATE_OUTPUT(" - start " << current_state);

        if (params_.frame_rate != 0.0f) {
          cv::Mat out_img = base_image_.clone();
          evaluation_model->drawState(current_state, out_img);
          cv::imshow("out", out_img);
        }

        solver->performEpisodeStartComputation(current_state);

        float distance_limit_pxl = ((float)params_.distance_limit) / map_.info.resolution;
        bool terminal = false;

        while (!terminal && instance_distance <= distance_limit_pxl) {

          Action action = solver->getBestAction(current_state);;
          EVALUATE_OUTPUT("   action: " << action);

          float transition_reward;
          State next_state;
          int depth_count;
          float time_loss, utility_loss;
          std::vector<State> frame_vector;
          evaluation_model->takeAction(current_state, action, transition_reward, next_state, terminal, depth_count,
                                       evaluation_rng, time_loss, utility_loss, frame_vector);
          float transition_distance = 
            bwi_mapper::getEuclideanDistance(next_state.graph_id, current_state.graph_id, graph_);

          instance_distance += transition_distance;
          instance_reward += transition_reward;
          instance_time += time_loss;
          instance_utility -= utility_loss;

          current_state = next_state;

          // Perform an MCTS search after next state is decided (not perfect)
          // Only perform search if system is left with any future action choice
          float time = (transition_distance * map_.info.resolution) / params_.human_speed;
          if (!terminal) {
            EVALUATE_OUTPUT("  Performing post-action computation for " << time << " seconds.");
          }

          // TODO we should do the computation with the original state and selected action.
          if (params_.frame_rate != 0.0f) {
            for (int time_step = 0; time_step < frame_vector.size(); ++time_step) {
              cv::Mat out_img = base_image_.clone();
              if (!terminal) {
                solver->performPostActionComputation(current_state, 1.0f / params_.frame_rate);
              } else {
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000.0f / params_.frame_rate));
              }
              evaluation_model->drawState(frame_vector[time_step], out_img);
              cv::imshow("out", out_img);
            }
          } else {
            if (!terminal) {
              solver->performPostActionComputation(current_state, time);
            }
          }

          EVALUATE_OUTPUT(" - transition " << current_state);

          record["reward"] = boost::lexical_cast<std::string>(instance_reward);
          record["distance"] = boost::lexical_cast<std::string>(instance_distance * map_.info.resolution);
          record["time"] = boost::lexical_cast<std::string>(instance_time);
          record["utility"] = boost::lexical_cast<std::string>(instance_time);

          // Produce normalized distance results as well.
          std::vector<size_t> temp_path;
          float normalization_distance = 
            bwi_mapper::getShortestPathWithDistance(start_idx, goal_idx, temp_path, graph_);
          float normalization_time = normalization_distance / params_.human_speed;
          record["normalized_distance"] = boost::lexical_cast<std::string>(instance_distance / normalization_distance);
          record["normalized_reward"] = boost::lexical_cast<std::string>(instance_distance / normalization_time);
          record["normalized_time"] = boost::lexical_cast<std::string>(instance_distance / normalization_time);
          record["normalized_utility"] = boost::lexical_cast<std::string>(instance_distance / normalization_time);

          records.push_back(record);
        }
      }

      bwi_tools::writeRecordsAsCSV(base_directory_ + "/results/part." + boost::lexical_cast<std::string>(seed),
                                   records);

    }
    
  } /* mrn */

} /* bwi_guidance_solver */

PLUGINLIB_EXPORT_CLASS(bwi_guidance_solver::mrn::Domain, bwi_rl::Domain)
