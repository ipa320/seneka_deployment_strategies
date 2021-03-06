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
 * Date of creation: October 2014
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

#ifndef SENSOR_PLACEMENT_HYPERVISOR_H
#define SENSOR_PLACEMENT_HYPERVISOR_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Path.h>

#include <seneka_sensor_placement/quanjo_maneuver.h>
#include <std_srvs/Empty.h>
#include <sstream>


class sensor_placement_hypervisor
{
  private:

  // nav_msgs::Path for saving all nodes that have been deployed
  nav_msgs::Path deployed_nodes_path_;

  public:

  // sensor_placement_hypervisor node handle
  ros::NodeHandle nh_;

  //publisher for topic "quanjo_maneuver"
  ros::Publisher quanjo_maneuver_pub_;

  //subscriber for topic "sensor_poses"
  ros::Subscriber nav_path_sub_;

  // service server for sensor_nodes_pickup service
  ros::ServiceServer ss_pickup_sensor_nodes_;

  // standard constructor
  sensor_placement_hypervisor();

  // standard destructor
  ~sensor_placement_hypervisor();

  // callback function for nav_path_sub_ subscriber
  void navPathSubCB(const nav_msgs::Path new_path);

  // callback function for pickup_sensor_nodes
  bool srvCB_pickup_sensor_nodes(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

};

#endif //SENSOR_PLACEMENT_HYPERVISOR_H
