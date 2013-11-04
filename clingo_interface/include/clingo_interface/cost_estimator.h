#ifndef COST_ESTIMATOR_BH7RIRHF
#define COST_ESTIMATOR_BH7RIRHF

#include <ros/ros.h>
#include <stdexcept>

#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <clingo_interface/structures.h>

namespace clingo_interface {

  typedef std::pair<const int, float> IFPair;
  typedef std::pair<const int, int> IIPair;
  typedef std::pair<const int, std::map<int, float> > IIFPair;
  typedef std::pair<const int, std::map<int, int> > IIIPair;
  typedef std::pair<const std::string, std::map<int, std::map< int, float> > > SIIFPair;
  typedef std::pair<const std::string, std::map<int, std::map< int, int> > > SIIIPair;

  class CostEstimator {

    public:

      CostEstimator () {

        ros::NodeHandle nh, private_nh("~");

        std::vector<std::string> unavailable_parameters;
        std::string door_file;
        if (!(private_nh.getParam("door_file", door_file))) {
          unavailable_parameters.push_back("door_file");
        }
        if (!(private_nh.getParam("values_file", values_file_))) {
          unavailable_parameters.push_back("values_file");
        }
        if (!(private_nh.getParam("lua_file", lua_file_))) {
          unavailable_parameters.push_back("door_file");
        }
        private_nh.param("alpha", alpha_, 0.5);
        private_nh.param("use_exponential_weighting", use_exponential_weighting_, true);

        if (unavailable_parameters.size() != 0) {
          std::string message = "Following neccessary params not available: " +
            boost::algorithm::join(unavailable_parameters, ", ");
          ROS_INFO_STREAM(message);
          throw std::runtime_error(message);
        }

        readDoorFile(door_file, doors_);
        prepareInputData();
      }

      void prepareInputData() {
        // Compute Iteration number
        iteration_ = 1;
        while (boost::filesystem::exists(values_file_ + 
              boost::lexical_cast<std::string>(iteration_))) {
          ++iteration_;
        }

        ROS_INFO_STREAM("ITERATION: " << iteration_);

        if (iteration_ == 1) {
          // Input data has no meaning. Initialize optimistically
          for (int idx = 0; idx < doors_.size(); ++idx) {
            for (int j = 0; j < doors_.size(); ++j) {
              if (j == idx) {
                continue;
              }
              if (doors_[j].approach_names[0] == doors_[idx].approach_names[0]) {
                std::string loc = doors_[j].approach_names[0];
                distance_estimates_[loc][idx][j] = 1.0f;
                distance_samples_[loc][idx][j] = 0;
              }
              if (doors_[j].approach_names[0] == doors_[idx].approach_names[1]) {
                std::string loc = doors_[j].approach_names[0];
                distance_estimates_[loc][idx][j] = 1.0f;
                distance_samples_[loc][idx][j] = 0;
              }
              if (doors_[j].approach_names[1] == doors_[idx].approach_names[0]) {
                std::string loc = doors_[j].approach_names[1];
                distance_estimates_[loc][idx][j] = 1.0f;
                distance_samples_[loc][idx][j] = 0;
              }
              if (doors_[j].approach_names[1] == doors_[idx].approach_names[1]) {
                std::string loc = doors_[j].approach_names[1];
                distance_estimates_[loc][idx][j] = 1.0f;
                distance_samples_[loc][idx][j] = 0;
              }
            }
          }
          // Write the lua file as it won't be available so that it can be used
          // in this turn
          writeValuesFile(0);
        } else {
          readValuesFile(iteration_ - 1);
        }
        writeLuaFile();
      }

      void writeLuaFile(std::string lua_file = "" ) {
        if (lua_file.empty()) {
          lua_file = lua_file_;
        }
        std::ofstream fout(lua_file.c_str());
        fout << "#begin_lua" << std::endl << std::endl;
        fout << "function dis(a,b,c)" << std::endl;
        fout << "\ta1 = tostring(a)" << std::endl;
        fout << "\tb1 = tostring(b)" << std::endl;
        fout << "\tc1 = tostring(c)" << std::endl;
        fout << "\tif a1==b1 then return 1 end" << std::endl;
        BOOST_FOREACH(SIIFPair const& kv, distance_estimates_) {
          fout << "\tif c1==\"" << kv.first << "\" then" << std::endl;
          BOOST_FOREACH(IIFPair const& kv2, kv.second) {
            fout << "\t\tif a1==\"" << doors_[kv2.first].name << "\" then" << std::endl;
            BOOST_FOREACH(IFPair const& kv3, kv2.second) {
              fout << "\t\t\tif b1==\"" << doors_[kv3.first].name << "\" then return " << lrint(kv3.second) << " end" << std::endl;
            }
            fout << "\t\tend" << std::endl;
          }
          fout << "\tend" << std::endl;
        }
        fout << "\treturn 1" << std::endl;
        fout << "end" << std::endl << std::endl;
        fout << "#end_lua." << std::endl;
        fout.close();
      }

      void writeValuesFile(int episode = -1) {
        if (episode == -1) {
          episode = iteration_;
        }
        std::string out_file_name = values_file_ + 
          boost::lexical_cast<std::string>(episode);
        std::ofstream fout(out_file_name.c_str());
        BOOST_FOREACH(SIIIPair const& kv, distance_samples_) {
          fout << " - name: " << kv.first << std::endl;
          fout << "   costs: " << std::endl;
          BOOST_FOREACH(IIIPair const& kv2, kv.second) {
            BOOST_FOREACH(IIPair const& kv3, kv2.second) {
              fout << "     - from: " << kv2.first << std::endl;
              fout << "       to: " << kv3.first << std::endl;
              fout << "       cost: " << distance_estimates_[kv.first][kv2.first][kv3.first] << std::endl;
              fout << "       samples: " << kv3.second << std::endl;
            }
          }
        }
        fout.close();
        std::string lua_file_name = values_file_ + "_distances" +
          boost::lexical_cast<std::string>(episode) + ".lua";
        writeLuaFile(lua_file_name);
      }

      void readValuesFile(int episode = -1) {
        if (episode == -1) {
          episode = iteration_;
        }
        std::string in_file_name = values_file_ + 
          boost::lexical_cast<std::string>(episode);
        std::ifstream fin(in_file_name.c_str());
        YAML::Parser parser(fin);

        YAML::Node doc;
        parser.GetNextDocument(doc);

        for (size_t i = 0; i < doc.size(); ++i) {
          std::string loc;
          doc[i]["name"] >> loc;
          const YAML::Node &costs = doc[i]["costs"];
          for (size_t j = 0; j < costs.size(); ++j) {
            int from, to;
            costs[j]["from"] >> from;
            costs[j]["to"] >> to;
            costs[j]["cost"] >> distance_estimates_[loc][from][to];
            costs[j]["samples"] >> distance_samples_[loc][from][to];
          }
        }
      }

      void addSample(std::string loc, int door_from, int door_to, float cost) {
        ROS_INFO_STREAM(std::string("Adding sample ") << doors_[door_from].name << "->" <<
            doors_[door_to].name << ": " << cost);
        int samples = distance_samples_[loc][door_from][door_to];
        float old_cost = distance_estimates_[loc][door_from][door_to];
        float final_cost = 0;
        if (use_exponential_weighting_) {
          final_cost = (1.0f - alpha_) * old_cost + alpha_ * cost; 
        } else {
          if (samples != 0) {
            final_cost = (old_cost * samples + cost) / (samples + 1);
          } else {
            // First sample
            final_cost = cost;
          }
        }
        distance_estimates_[loc][door_from][door_to] = final_cost;
        distance_estimates_[loc][door_to][door_from] = final_cost;
        distance_samples_[loc][door_from][door_to] = samples + 1;
        distance_samples_[loc][door_to][door_from] = samples + 1;
      }

      void finalizeEpisode() {
        writeLuaFile();
        writeValuesFile();
        ++iteration_;
        ROS_INFO_STREAM("Bumping to ITERATION #" << iteration_);
      }

    private:

      std::vector<clingo_interface::Door> doors_;
      std::map<std::string, std::map<int, std::map<int, float> > > distance_estimates_;
      std::map<std::string, std::map<int, std::map<int, int> > > distance_samples_;

      std::string values_file_;
      std::string lua_file_;

      double alpha_;
      bool use_exponential_weighting_;
      int iteration_;
  };
}

#endif /* end of include guard: COST_ESTIMATOR_BH7RIRHF */
