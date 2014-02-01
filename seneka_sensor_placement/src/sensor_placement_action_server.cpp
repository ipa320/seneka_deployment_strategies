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


#include <sensor_placement_action_server.h>

//constructor
sensorPlacementAction::sensorPlacementAction(std::string name)
:
  //initializer list
  //action_server takes arguments of a node handle, name of the action, optionally an executeCB
  as_(nh_, name, boost::bind(&sensorPlacementAction::executeCB, this, _1), false), // -b-_1 ??
  action_name_(name)
  {
    as_.start();
  }

//destructor
sensorPlacementAction::~sensorPlacementAction(void){}

  //callback function is passed a pointer to the message
  //Note: This is a boost shared pointer, given by appending "ConstPtr" to the end of the goal message type.
void sensorPlacementAction::executeCB(const seneka_sensor_placement::sensorPlacementGoalConstPtr &goal)
{
  // helper variables
  ros::Rate r(1);   //-b- why needed??
  bool success = true;

  switch (goal->service_id)
  {
    case 1:
    {
      //startPSO
      ROS_INFO("Starting PSO action");

      // check that preempt has not been requested by the client    -b- !important step for preemption
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;

        // -b- NOTE:  Setting the rate at which the action server checks for preemption requests is left to the implementor of the server.
      }

      //do stuff


      break;
    }


    default:
    {
      ROS_ERROR("invlaid service_id");
    }

  }

  if(success)
  {
    //show results here -b-
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    as_.setSucceeded(result_);
  }








}


//main program
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensorPlacementActionServer");   //-b- suitable name ?

  sensorPlacementAction sensorPlacementActionServer(ros::this_node::getName());
  ros::spin();

  return 0;
}


/*

  //old executeCB for
  void executeCB(const seneka_sensor_placement::testGoalConstPtr &goal)
  {

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    if (goal->startPSO == true)
    ROS_INFO("startPSO goal recieved");

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating test sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client    -b- !important step for preemption
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;

        // -b- NOTE:  Setting the rate at which the action server checks for preemption requests is left to the implementor of the server.
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback **
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


*/





