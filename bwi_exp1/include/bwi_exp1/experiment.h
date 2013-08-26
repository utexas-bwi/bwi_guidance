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

#include <bwi_exp1/structures.h>

namespace bwi_exp1 {

  struct Instance {
    Location start_loc;
    Location ball_loc;
    float max_duration;
  };

  struct InstanceGroup {
    std::string prefix;
    int order;
    std::vector<Instance> experiments;
  };

  struct Experiment {
    std::string person_id;
    std::vector<Robot> robots;
    std::string ball_id;
    std::vector<InstanceGroup> experiments; 
  };

  void readExperimentFromFile(const std::string& file, 
      Experiment& ec); 

  void getInstanceNames(const Experiment& ec, 
      std::vector<std::string> &names); 

  void computeOrderings(const Experiment& ec,
      std::vector< std::vector<std::string> >& orderings); 

  Instance& getInstance(Experiment& ec, size_t idx);

  Instance& getInstance(Experiment& ec, std::string instance_name);
  
} /* bwi_exp1 */

#endif /* end of include guard: EXPERIMENT_500N2RKG */
