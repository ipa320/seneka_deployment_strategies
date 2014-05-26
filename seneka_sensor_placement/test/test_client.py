#!/usr/bin/env python
#################################################################
##\file
#
# \note
#   Copyright (c) 2014 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Repository name: seneka_deployment_strategies
# \note
#   ROS package name: seneka_sensor_placement
#
# \author
#   Author: Matthias Gruhler, email:Matthias.Gruhler@ipa.fgh.de
#
# \date Date of creation: May 2014
#
# \brief
#   Test client for automatically triggering and testing the sensor placment node.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import roslib
import rospy

import actionlib
import dynamic_reconfigure.client

import datetime

from std_srvs.srv import Empty
from seneka_sensor_placement.srv import polygon_offset
import seneka_sensor_placement.msg

# lists specifying the different parameters, over which to iterate
global gs_resolution, greedyPSO_particles, number_of_sensors
gs_resolution = [10.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.75, 0.5, 0.25, 0.1]
greedyPSO_particles = [100, 40, 30, 20, 10]
number_of_sensors = [5, 7]

# IDs for action calls
global algo
algo = {"PSO": 1, "GreedyPSO": 2, "GreedySearch": 3, "GreedySearchOffset": 4}


def update_config(client, cfg_update):
    rospy.sleep(2.5) # sleep some time to wait for action server
    old_cfg = dyn_reconf_client.get_configuration(5.0)
    dyn_reconf_client.update_configuration(cfg_update)
    new_cfg = dyn_reconf_client.get_configuration(5.0)
    rospy.sleep(2.5)
    return old_cfg, new_cfg

def call_action(client, goal):
    rospy.sleep(2.5)
    start_time = rospy.Time().now()
    client.wait_for_server()
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    end_time = rospy.Time().now()
    delta_t = (end_time - start_time).to_sec()
    rospy.sleep(2.5)
    return delta_t, result

if __name__ == '__main__':
    # init node
    rospy.init_node("sensor_placement_test_node")
    rospy.sleep(0.5)

    # setup client
    dyn_reconf_client = dynamic_reconfigure.client.Client("/sensor_placement")
    # get current config
    sensor_placement_config = dyn_reconf_client.get_configuration(5.0)
    rospy.loginfo("Dynamic reconfigure Client set up!")

    # setup action client
    client = actionlib.SimpleActionClient('sensorPlacementActionServer',seneka_sensor_placement.msg.sensorPlacementAction)
    # setup dummy goal
    goal = seneka_sensor_placement.msg.sensorPlacementGoal()


    # setup services for publishing different stuff
    rospy.wait_for_service('publish_AoI')
    pub_AoI_sc = rospy.ServiceProxy('publish_AoI', Empty)
    rospy.loginfo("publish_AoI Service connected!")

    rospy.wait_for_service('publish_PoI')
    pub_PoI_sc = rospy.ServiceProxy('publish_PoI', Empty)
    rospy.loginfo("publish_PoI Service connected!")

    rospy.wait_for_service('publish_PoIs')
    pub_PoIs_sc = rospy.ServiceProxy('publish_PoIs', Empty)
    rospy.loginfo("publish_PoIs Service connected!")

    rospy.wait_for_service('publish_forbidden_areas')
    pub_forbidden_area_sc = rospy.ServiceProxy('publish_forbidden_areas', Empty)
    rospy.loginfo("publish_forbidden_areas Service connected!")


    ##
    ## run tests
    ##
    now = datetime.datetime.now()
    string_to_file = "Results of test run at "+str(now)+"\n===================\n\n"
    f.write(string_to_file)

    ## setup goal
    rospy.loginfo("Creating Area of interest")
    pub_AoI_sc()

    update = {"max_num_iterations": str(10)}
    old_cfg, new_cfg = update_config(dyn_reconf_client, update)

    # open file
    with open('testoutput.txt','w') as f:
        for num in number_of_sensors:
            rospy.loginfo("Set new configuration")
            update = {"number_of_sensors": str(num)}
            old_cfg, new_cfg = update_config(dyn_reconf_client, update)

            # call action
            goal.service_id = algo["PSO"]
            delta_t, result = call_action(client,goal)

            for key, service_id in algo.iteritems():
                if service_id == goal.service_id:
                    string_to_file = "Called Action with Algo "+str(key)+"\n"
            f.write(string_to_file)
            string_to_file = "Updated configuration is "+str(update)+"\n"
            f.write(string_to_file)
            string_to_file = "Returned with coverage "+str(result)+" in "+str(delta_t)+" seconds"+"\n=================\n"
            f.write(string_to_file)


    
