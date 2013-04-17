/**
 * \file  users.h
 * \brief  Some functions and data structures for parsing user data
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
 * $ Id: 04/17/2013 02:30:57 PM piyushk $
 *
 **/

#ifndef USERS_O0660P4W
#define USERS_O0660P4W

#include<stdint.h>

namespace bwi_exp1 {
  
  struct User {
    std::string id;
    std::string name;
    std::vector<std::string> experiment_group_order;
    std::vector<size_t> experiment_group_size;
    uint8_t color[3];
  };

  void operator >> (const YAML::Node& node, User& user) {
    node["user"] >> user.id;
    node["name"] >> user.name;
    user.experiment_group_order.clear();
    user.experiment_group_size.clear();
    for (size_t i = 0; i < node["experiment_order"].size(); ++i) {
      std::string experiment_group_name;
      size_t experiment_group_size;
      node["experiment_order"][i]["name"] >> experiment_group_name;
      node["experiment_order"][i]["num"] >> experiment_group_size;
      user.experiment_group_order.push_back(experiment_group_name);
      user.experiment_group_size.push_back(experiment_group_size);
    }
  }

  void readUserDataFromFile(const std::string& file, std::vector<User>& users) {
    std::ifstream fin(file);
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);
    for (size_t i = 0; i < doc.size(); ++i) {
      User user;
      doc[i] >> user;
      users.push_back(user);
    }
  }

  std::vector<size_t> getAllUserIds(const std::vector<User>& users) {
    std::vector<size_t> user_ids;
    for (size_t i = 0; i < users.size(); ++i) {
      user_ids.push_back(user_ids);
    }
  }

  std::vector<size_t> getUserIdsForOrdering(const std::vector<User>& users,
      std::vector<std::string> ordering) {
    std::vector<size_t> user_ids;
    for (size_t i = 0; i < users.size(); ++i) {
      if (std::equal(ordering.begin(), ordering.end(), 
            users[i].experiment_group_order.begin())) {
        users_ids.push_back(i);
    }
  }
  
} /* bwi_exp1 */

#endif /* end of include guard: USERS_O0660P4W */
