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


#ifndef SENSOR_PLACEMENT_INTERFACE_H
#define SENSOR_PLACEMENT_INTERFACE_H

// ros includes
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// ros msg/srv include includes
#include <seneka_sensor_placement/polygon_offset.h>
#include <seneka_sensor_placement/sensorPlacementAction.h>

class sensor_placement_interface
{
  private:
  // create the action client
  actionlib::SimpleActionClient<seneka_sensor_placement::sensorPlacementAction> ac_;

  public:
  // standard constructor
  sensor_placement_interface();
  // standard destructor
  ~sensor_placement_interface();

  // create node handles
  ros::NodeHandle nh_;

  // declaration of ros service servers
  ros::ServiceServer ss_start_PSO_;
  ros::ServiceServer ss_start_GreedyPSO_;
  ros::ServiceServer ss_start_GS_;
  ros::ServiceServer ss_start_GS_with_offset_;
  ros::ServiceServer ss_clear_fa_vec_;
  ros::ServiceServer ss_clear_PoI_vec_;
  ros::ServiceServer ss_cancel_action_;

  /* ------------------------------------------- */
  /* --------- ROS Service Callbacks ----------- */
  /* ------------------------------------------- */

  // callback function for the start PSO service
  bool startPSOCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // callback function for the start GreedyPSO service
  bool startGreedyPSOCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // callback function for the start GS service
  bool startGSCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // callback function for the start GS service with offset parameter
  bool startGSWithOffsetCallback(seneka_sensor_placement::polygon_offset::Request& req, seneka_sensor_placement::polygon_offset::Response& res);

  // callback function for clearing all forbidden areas
  bool clearFAVecCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // callback function for clearing all points of interest
  bool clearPoIVecCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // callback function for the cancelling active service
  bool cancelGoalCallBack(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

};


#endif
