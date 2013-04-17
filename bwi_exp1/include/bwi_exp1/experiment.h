/**
 * \file  experiment.h
 * \brief  File containing some useful c++ functions and data structures for
 *         parsing experiment data
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 *
 * Copyright (c) 2013, UT Austin

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *
 * $ Id: 04/17/2013 01:22:57 PM piyushk $
 *
 **/

#ifndef EXPERIMENT_500N2RKG
#define EXPERIMENT_500N2RKG

#include <string>
#include <vector>
#include <algorithm>
#include <yaml-cpp/yaml.h>

namespace bwi_exp1 {

  struct ExperimentLocation {
    float x;
    float y;
    float yaw;
  };

  struct ExperimentRobot {
    std::string id;
    ExperimentLocation default_loc;
  };

  struct ExperimentPathPoint {
    size_t graph_id;
    bool robot_present;
  };

  struct Experiment {
    ExperimentLocation start_loc;
    ExperimentLocation ball_loc;
    std::vector<ExperimentPathPoint> path;
    std::vector<ExperimentLocation> extra_robots;
    float max_duration;
  };

  struct ExperimentGroup {
    std::string prefix;
    int order;
    std::vector<Experiment> experiments;
  };

  struct ExperimentCollection {
    std::string person_id;
    std::vector<ExperimentRobot> robots;
    std::string ball_id;
    std::vector<ExperimentGroup> experiments; 
  };

  void operator >> (const YAML::Node& node, Experiment& exp) {
    node["start_x"] >> exp.start_loc.x;
    node["start_y"] >> exp.start_loc.y;
    node["start_yaw"] >> exp.start_loc.yaw;
    node["ball_x"] >> exp.ball_loc.x;
    node["ball_y"] >> exp.ball_loc.y;
    exp.path.clear();
    for (size_t i = 0; i < node["path"].size(); ++i) {
      ExperimentPathPoint path_point;
      node["path"][i] >> path_point;
      exp.path.push_back(path_point);
    }
  }

  void operator >> (const YAML::Node& node, ExperimentGroup& eg) {
    node["prefix"] >> eg.prefix;
    node["order"] >> eg.order;
    eg.experiments.clear();
    for (size_t i = 0; i < node["experiments"].size(); ++i) {
      Experiment exp;
      node["experiments"][i] >> exp;
      eg.experiments.push_back(exp);
    }
  }

  void operator >> (const YAML::Node& node, ExperimentRobot& robot) {
    node["id"] >> robot.id;
    node["default_loc"][0] >> robot.default_loc.x;
    node["default_loc"][1] >> robot.default_loc.y;
    robot.default_loc.z = 0;
  }

  void operator >> (const YAML::Node& node, ExperimentCollection& ec) {
    node["person_id"] >> ec.person_id;
    ec.robots.clear();
    for (size_t i = 0; i < node["robots"].size(); ++i) {
      ExperimentRobot robot;
      node["robots"][i] >> robot;
      ec.robots.push_back(robot);
    }
    ec.experiments.clear();
    for (size_t i = 0; i < node["experiments"].size(); ++i) {
      ExperimentGroup group;
      node["experiments"][i] >> group;
      ec.experiments.push_back(group);
    }
    node["ball_id"] >> ec.person_id;
  }

  void readExperimentCollectionFromFile(const std::string& file, 
      ExperimentCollection& ec) {

    std::ifstream fin(file);
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);
    doc >> ec;
  }

  void getExperimentNames(const ExperimentCollection& ec, 
      std::vector<std::string> &names) {
    names.clear();
    for (size_t i = 0; i < ec.experiments.size(); ++i) {
      const ExperimentGroup& eg = ec.experiments[i];
      for (size_t j = 0; j < ej.experiments.size(); ++j) {
        names.push_back(ec.prefix + "_" + 
            boost::lexical_cast<std::string>(j));
      }
    }
  }

  void computeOrderings(const ExperimentCollection& ec,
      std::vector< std::vector<std::string> >& orderings) {

    std::vector<std::string> base_order;
    base_order.resize(ec.experiments.size());
    std::vector<std::string> randomly_ordered_groups;
    for (size_t i = 0; i < ec.experiments.size(); ++i) {
      const ExperimentGroup& eg = ec.experiments[i];
      if (eg.order != -1) {
        base_order[eg.order] = eg.prefix;
      } else {
        randomly_ordered_groups.push_back(eg.prefix);
      }
    }

    // sort the randomly ordered groups into the lowest permutation
    std::sort(randomly_ordered_groups.begin(), randomly_ordered_groups.end());

    // Compute each permutation
    orderings.clear();
    do {
      std::vector<std::string> current_permutation;
      size_t base_order_size = 
        base_order.size() - randomly_ordered_groups.size();
      current_permutation.insert(current_permutation.end(), base_order.begin(),
          base_order.begin() + base_order_size);
      current_permutation.insert(current_permutation.end(),
          randomly_ordered_groups.begin(), randomly_ordered_groups.end());
      orderings.insert(current_permutation);
    } while (std::next_permutation(randomly_ordered_groups.begin(), 
          randomly_ordered_groups.end());

  }
  
} /* bwi_exp1 */

#endif /* end of include guard: EXPERIMENT_500N2RKG */
