/****************************************************************
 *
 * Copyright (c) 2013
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


//##############################################
//########### This is action client ############
//## here, services are responded and goals   ##
//## are sent to the action server            ##
//##############################################


sensor_placement_interface::sensor_placement_interface()
:ac_("sensorPlacementActionServer", true)
{
  ss_start_PSO_ = nh_.advertiseService("StartPSO_ac", &sensor_placement_interface::startPSOCallback, this);
  ss_cancel_action_ = nh_.advertiseService("CancelAction", &sensor_placement_interface::cancelGoalCallBack, this);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac_.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, ready to send goals");


}

// destructor
sensor_placement_interface::~sensor_placement_interface(){}


bool sensor_placement_interface::startPSOCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("PSO service call received");
    // send a goal to the action
  seneka_sensor_placement::sensorPlacementGoal goal;
  goal.service_id = 1;
  ac_.sendGoal(goal);
  return true;
}

bool sensor_placement_interface::cancelGoalCallBack(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("Cancel action request received");
 /* // send a goal to the action
  seneka_sensor_placement::sensorPlacementGoal goal;
  goal.cancel_service = true;
  ac_.sendGoal(goal);
*/
ac_.cancelGoal();
return true;
}




int main (int argc, char **argv)
{
  ros::init(argc, argv, "sensor_placement_interface");

  sensor_placement_interface my_interface;
/*
  // send a goal to the action
  seneka_sensor_placement::sensorPlacementGoal goal;
  goal.service_id = 1;
  ac_.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac_.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac_.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  //exit
*/
  ros::spin(); // -b- needed?
  return 0;
}



