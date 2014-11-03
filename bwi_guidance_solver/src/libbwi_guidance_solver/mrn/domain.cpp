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
      params_.robot_home_base_file = bwi_tools::resolveRosResource(params_.robot_home_base_file);

      // Read the map and graph file and the home bases of the robots.
      if (boost::filesystem::is_regular_file(params_.map_file) &&
          boost::filesystem::is_regular_file(params_.graph_file) &&
          boost::filesystem::is_regular_file(params_.robot_home_base_file)) {
        // Read map and graph
        bwi_mapper::MapLoader mapper(params_.map_file);
        mapper.getMap(map_);
        mapper.drawMap(base_image_);
        bwi_mapper::readGraphFromFile(params_.graph_file, map_.info, graph_);
        readRobotHomeBase(params_.robot_home_base_file, robot_home_base_);
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
        "-tl" << params_.time_limit << 
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
          if (!(solver->initialize(params_, solvers[solver_idx]["params"], map_, graph_, robot_home_base_, base_directory_))) {
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
      int start_robot_idx = rng.randomInt(robot_home_base_.size() - 1);
      int start_idx = (params_.start_colocated) ? robot_home_base_[start_robot_idx] : rng.randomInt(num_vertices - 1);
      int goal_idx = rng.randomInt(num_vertices - 1);
      while (goal_idx == start_idx) {
        goal_idx = rng.randomInt(num_vertices - 1);
      }

      float shortest_distance = bwi_mapper::getShortestPathDistance(start_idx, goal_idx, graph_) * map_.info.resolution;
      std::cout << "Testing instance with start_idx: " << start_idx << ", goal_idx: " << goal_idx << " and shortest distance " << shortest_distance << std::endl;
      
      // We can create the model right now as loading the model is not dependent on the method parameters. We just need
      // to make sure we reinitialize the rng and update the reward structure for every method as necessary
      // Initialize the transition model.
      MotionModel::Ptr motion_model(new MotionModel(graph_, 
                                                    params_.robot_speed / map_.info.resolution,
                                                    params_.human_speed / map_.info.resolution)); 
      TaskGenerationModel::Ptr task_generation_model(new TaskGenerationModel(robot_home_base_, 
                                                                             graph_, 
                                                                             params_.utility_multiplier));
      HumanDecisionModel::Ptr human_decision_model(new HumanDecisionModel(graph_));

      // Set the MDP parameters and initialize the MDP.
      PersonModel::Params mdp_params;
      mdp_params.frame_rate = params_.frame_rate; // This version of the model should never visualize, as it is used for sampling only.
      mdp_params.num_robots = robot_home_base_.size();
      mdp_params.avg_robot_speed = params_.robot_speed;
      boost::shared_ptr<PersonModel> evaluation_model(new PersonModel(graph_, 
                                                                      map_, 
                                                                      goal_idx, 
                                                                      motion_model,
                                                                      human_decision_model,
                                                                      task_generation_model,
                                                                      mdp_params));

      std::vector<std::map<std::string, std::string> > records;
      BOOST_FOREACH(boost::shared_ptr<Solver>& solver, solvers_) {

        EVALUATE_OUTPUT("Evaluating solver " << solver->getSolverName());
        solver->reset(seed, goal_idx);
        std::map<std::string, std::string> record = solver->getParamsAsMap();

        record["name"] = solver->getSolverName();
        record["start_idx"] = boost::lexical_cast<std::string>(start_idx);
        record["goal_idx"] = boost::lexical_cast<std::string>(goal_idx);

        boost::shared_ptr<RNG> evaluation_rng(new RNG(2 * (seed + 1)));

        // Initialize the start state.
        State current_state; 
        current_state.loc_node = start_idx;
        current_state.loc_p = 0.0f;
        current_state.loc_prev = start_idx;
        current_state.assist_type = NONE;
        current_state.assist_loc = NONE;
        current_state.robots.resize(robot_home_base_.size());
        int robot_counter = 0;
        BOOST_FOREACH(RobotState &robot, current_state.robots) {
          robot.loc_u = robot_home_base_[robot_counter];
          robot.loc_p = 0.0f;
          robot.loc_v = robot.loc_u;
          robot.help_destination = NONE;
          task_generation_model->generateNewTaskForRobot(robot_counter, robot, *evaluation_rng);
          ++robot_counter;
        }

        if (params_.start_colocated) {
          // If the robot starts colocated as the human, then assign that robot to help the human.
          float unused_reward;
          int unused_depth_count;
          bool unused_terminal;
          State next_state;
          evaluation_model->takeAction(current_state, Action(ASSIGN_ROBOT, start_robot_idx, start_idx), 
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
          // TODO replace call with something that produces the video as well.
          cv::imshow("out", out_img);
        }

        solver->performEpisodeStartComputation(current_state);

        bool terminal = false;

        while (!terminal && instance_time <= params_.time_limit) {

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
            bwi_mapper::getEuclideanDistance(next_state.loc_node, current_state.loc_node, graph_);

          instance_distance += transition_distance;
          instance_reward += transition_reward;
          instance_time += time_loss;
          instance_utility -= utility_loss;

          current_state = next_state;

          // Perform an MCTS search after next state is decided (not perfect)
          // Only perform search if system is left with any future action choice
          float time = time_loss;
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
              // TODO replace call with something that produces the video as well.
              cv::imshow("out", out_img);
            }
          } else {
            if (!terminal) {
              solver->performPostActionComputation(current_state, time);
            }
          }

          EVALUATE_OUTPUT(" - transition " << current_state);
        }

        record["reward"] = boost::lexical_cast<std::string>(instance_reward);
        record["distance"] = boost::lexical_cast<std::string>(instance_distance * map_.info.resolution);
        record["time"] = boost::lexical_cast<std::string>(instance_time);
        record["utility"] = boost::lexical_cast<std::string>(instance_utility);

        // Produce normalized distance results as well.
        float normalization_distance = 
          bwi_mapper::getShortestPathDistance(start_idx, goal_idx, graph_);
        float normalization_time = map_.info.resolution * normalization_distance / params_.human_speed;
        record["normalized_distance"] = boost::lexical_cast<std::string>(instance_distance / normalization_distance);
        record["normalized_reward"] = boost::lexical_cast<std::string>(instance_reward / normalization_time);
        record["normalized_time"] = boost::lexical_cast<std::string>(instance_time / normalization_time);
        record["normalized_utility"] = boost::lexical_cast<std::string>(instance_utility / normalization_time);

        records.push_back(record);
        
      }

      bwi_tools::writeRecordsAsCSV(base_directory_ + "/results/part." + boost::lexical_cast<std::string>(seed),
                                   records);

    }
    
  } /* mrn */

} /* bwi_guidance_solver */

PLUGINLIB_EXPORT_CLASS(bwi_guidance_solver::mrn::Domain, bwi_rl::Domain)
