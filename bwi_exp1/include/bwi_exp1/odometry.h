/**
 * \file  odometry.h
 * \brief  Reads odometry data from individual odometry runs
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
 * $ Id: 04/18/2013 10:18:57 AM piyushk $
 *
 **/

#ifndef ODOMETRY_LVMM1B22
#define ODOMETRY_LVMM1B22

#include <bwi_exp1/experiment.h>

namespace bwi_exp1 {

  struct ExperimentLocationStamped {
    ExperimentLocation location;
    float seconds_since_start;
  };

  bool readOdometry(const std::string& file, 
      std::vector<ExperimentLocationStamped>& path) {

    std::ifstream fin(file);
    if (!file) {
      path.clear();
      return false;
    }
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);

    float start_time = 0;

    path.clear();
    for (size_t i = 0; i < doc["odometry"].size(); ++i) {
      const YAML::Node& stamped_location = doc["odometry"][i];
      ExperimentLocation location;
      location.location.x = stamped_location[0];
      location.location.y = stamped_location[1];
      int stamp_idx = 2;
      if (stamped_location.size() == 4) {
        location.location.yaw = stamped_location[2];
        stamp_idx = 3;
      } else {
        location.location.yaw = 0;
      }
      if (i == 0) {
        start_time = stamped_location[stamp_idx];
      }
      location.seconds_since_start = stamped_location[stamp_idx] - start_time;
      path.push_back(location);
    }

    return doc["success"];
  }
  
} /* bwi_exp1 */

#endif /* end of include guard: ODOMETRY_LVMM1B22 */

