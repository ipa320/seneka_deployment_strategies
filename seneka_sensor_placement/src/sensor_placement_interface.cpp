/****************************************************************
 *
 * Copyright (c) 2014
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: SeNeKa
 * ROS stack name: seneka_deployment_strategies
 * ROS package name: seneka_sensor_placement
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Muhammad Bilal Chughtai, email:Muhammad.Chughtai@ipa.fraunhofer.de
 *
 * Date of creation: January 2014
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Fraunhofer Institute for Manufacturing
 *     Engineering and Automation (IPA) nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <sensor_placement_interface.h>


//############################################
//######### This is action client ############
//## here, services are responded and goals ##
//## are sent to the action server          ##
//############################################


sensor_placement_interface::sensor_placement_interface()
:ac_("sensorPlacementActionServer", true)
{
  // ros service servers
  ss_start_PSO_ = nh_.advertiseService("startPSO", &sensor_placement_interface::startPSOCallback, this);
  ss_start_GreedyPSO_ = nh_.advertiseService("startGreedyPSO", &sensor_placement_interface::startGreedyPSOCallback, this);
  ss_start_GS_ = nh_.advertiseService("startGS", &sensor_placement_interface::startGSCallback, this);
  ss_start_GS_with_offset_ = nh_.advertiseService("startGS_with_offset_polygon", &sensor_placement_interface::startGSWithOffsetCallback, this);
  ss_clear_fa_vec_ = nh_.advertiseService("clear_forbidden_areas", &sensor_placement_interface::clearFACallback, this);
  ss_test_ = nh_.advertiseService("testService", &sensor_placement_interface::testServiceCallback, this);
  ss_cancel_action_ = nh_.advertiseService("cancel_action", &sensor_placement_interface::cancelGoalCallBack, this);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac_.waitForServer();
  ROS_INFO("Action server started, ready to send goals");
}

// destructor
sensor_placement_interface::~sensor_placement_interface(){}

// callback function for the start PSO service
bool sensor_placement_interface::startPSOCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("'PSO' service call received");
  // send goal
  seneka_sensor_placement::sensorPlacementGoal goal;
  goal.service_id = 1;
  ac_.sendGoal(goal);
  return true;
}

// callback function for the start GreedyPSO service
bool sensor_placement_interface::startGreedyPSOCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("'GreedyPSO' service call received");
  // send goal
  seneka_sensor_placement::sensorPlacementGoal goal;
  goal.service_id = 2;
  ac_.sendGoal(goal);
  return true;
}

// callback function for the start GS service
bool sensor_placement_interface::startGSCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("'GreedySearch' service call received");
  // send goal
  seneka_sensor_placement::sensorPlacementGoal goal;
  goal.service_id = 3;
  ac_.sendGoal(goal);
  return true;
}

// callback function for the start GS service with offset parameter
bool sensor_placement_interface::startGSWithOffsetCallback(seneka_sensor_placement::polygon_offset::Request& req, seneka_sensor_placement::polygon_offset::Response& res)
{
  ROS_INFO("'GreedySearch_with_offset_polygon' service call received");
  // send goal
  seneka_sensor_placement::sensorPlacementGoal goal;
  goal.service_id = 4;
  goal.service_input_arg = req.offset_value;
  ac_.sendGoal(goal);
  res.success=true;
  return true;
}

// callback function for clearing all forbidden areas
bool sensor_placement_interface::clearFACallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("'Clear_forbidden_areas' service call received");
  // send goal
  seneka_sensor_placement::sensorPlacementGoal goal;
  goal.service_id = 5;
  ac_.sendGoal(goal);
  return true;
}

bool sensor_placement_interface::testServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("TestService call received");
  // send goal
  seneka_sensor_placement::sensorPlacementGoal goal;
  goal.service_id = 6;
  ac_.sendGoal(goal);
  return true;
}

bool sensor_placement_interface::cancelGoalCallBack(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("Cancel action request received");
  ac_.cancelGoal();
  return true;
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "sensor_placement_interface");

  sensor_placement_interface my_interface;

  ros::spin();
  return 0;
}



