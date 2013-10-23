#ifndef COST_ESTIMATOR_BH7RIRHF
#define COST_ESTIMATOR_BH7RIRHF

#include <ros/ros.h>
#include <stdexcept>

#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/shared_ptr.hpp>

#include <clingo_interface/structures.h>

namespace clingo_interface {

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
        private_nh.param("alpha", alpha_, 2.0/3.0);

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
          distance_estimates_.resize(doors_.size());
          for (int idx = 0; idx < doors_.size(); ++idx) {
            for (int j = 0; j < doors_.size(); ++j) {
              if (j == idx) {
                continue;
              }
              if (doors_[j].approach_names[0] == doors_[idx].approach_names[0] ||
                  doors_[j].approach_names[0] == doors_[idx].approach_names[1] ||
                  doors_[j].approach_names[1] == doors_[idx].approach_names[0] ||
                  doors_[j].approach_names[1] == doors_[idx].approach_names[1]) {
                distance_estimates_[idx][j] = 1.0f;
              }
            }
          }
          // Write the lua file as it won't be available so that it can be used
          // in this turn
          writeLuaFile();
          writeValuesFile(0);
        } else {
          readValuesFile(iteration_ - 1);
        }
      }

      void writeLuaFile() {
        std::ofstream fout(lua_file_.c_str());
        fout << "#begin_lua" << std::endl << std::endl;
        fout << "function dis(a,b)" << std::endl;
        fout << "\ta1 = tostring(a)" << std::endl;
        fout << "\tb1 = tostring(b)" << std::endl;
        fout << "\tif a1==b1 then return 1 end" << std::endl;
        for (int idx = 0; idx < doors_.size(); ++idx) {
          for (std::map<int,float>::iterator it = distance_estimates_[idx].begin();
              it != distance_estimates_[idx].end(); ++it) {
            fout << "\tif a1==\"" << doors_[idx].name << "\" and b1==\"" <<
              doors_[it->first].name << "\" then return " << 
              lrint(it->second) << " end" << std::endl;
          }
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
        for (int idx = 0; idx < doors_.size(); ++idx) {
          fout << " - name: " << doors_[idx].name << std::endl;
          fout << "   costs: " << std::endl;
          for (std::map<int,float>::iterator it = distance_estimates_[idx].begin();
              it != distance_estimates_[idx].end(); ++it) {
            fout << "     - to: " << it->first << std::endl;
            fout << "       cost: " << it->second << std::endl;
          }
        }
        fout.close();
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

        distance_estimates_.resize(doc.size());
        for (size_t i = 0; i < doc.size(); ++i) {
          const YAML::Node &costs = doc[i]["costs"];
          for (size_t j = 0; j < costs.size(); ++j) {
            int to;
            costs[j]["to"] >> to;
            costs[j]["cost"] >> distance_estimates_[i][to];
          }
        }
      }

      void addSample(int door_from, int door_to, float cost) {
        ROS_INFO_STREAM("Adding sample " << doors_[door_from].name << "->" <<
            doors_[door_to].name << ": " << cost);
        // Exponentially weighted average
        distance_estimates_[door_from][door_to] = alpha_ * cost + (1 - alpha_) *
          distance_estimates_[door_from][door_to];
      }

      void finalizeEpisode() {
        writeLuaFile();
        writeValuesFile();
      }

    private:

      std::vector<clingo_interface::Door> doors_;
      std::vector<std::map<int, float> > distance_estimates_;

      std::string values_file_;
      std::string lua_file_;

      double alpha_;
      int iteration_;
  };
}

#endif /* end of include guard: COST_ESTIMATOR_BH7RIRHF */
