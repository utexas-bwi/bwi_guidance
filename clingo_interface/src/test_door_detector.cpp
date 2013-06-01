/**
 * \file  test_door_detector.cpp
 * \brief  A small tester for the door state detector
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
 * $ Id: 05/06/2013 02:05:01 PM piyushk $
 *
 **/

#include <ros/ros.h>
#include <topological_mapper/map_loader.h>
#include <clingo_interface/door_handler.h>

int main(int argc, char *argv[]) {
  
  ros::init(argc, argv, "door_detector_test");
  ros::NodeHandle nh;

  std::string map_file, door_file, location_file;
  ros::param::get("~map_file", map_file);
  ros::param::get("~door_file", door_file);
  ros::param::get("~locations_file", location_file);

  clingo_interface::DoorHandler dd(map_file, door_file, location_file);

  ros::Rate rate(10);

  while (ros::ok()) {
    std::cout << "Door 0 is open: " << dd.isDoorOpen(0) << std::endl;
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
