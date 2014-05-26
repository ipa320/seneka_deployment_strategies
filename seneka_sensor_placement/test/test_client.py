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
# gs_resolution = [10.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.75, 0.5, 0.25, 0.1]
# greedyPSO_particles = [100, 40, 30, 20, 10]
# number_of_sensors = [1, 2, 3, 4, 5, 6, 7]

gs_resolution = [5.0]
greedyPSO_particles = [20]
number_of_sensors = [5]


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

def result_to_file(f, algo_id, update, result, delta_t, service_input_arg=None):
    for key, service_id in algo.iteritems():
        if service_id == algo_id:
            string_to_file = "Called Action with Algo "+str(key)+"\n"
            f.write(string_to_file)
    if not service_input_arg == None:
        string_to_file = "Set service_input_arg to "+str(service_input_arg)+"\n"
        f.write(string_to_file)
    string_to_file = "Updated configuration is "+str(update)+"\n"
    f.write(string_to_file)
    string_to_file = "Returned with result "+str(result)+" in "+str(delta_t)+" seconds"+"\n"
    f.write(string_to_file)
    #shortform
    string_to_file = "short: "+str(algo_id)+" "+str(result)+" "+str(delta_t)
    for key, value in update.iteritems():
        string_to_file += " "+str(key)+": "+str(value)
    if not service_input_arg == None:
        string_to_file += " "+str(service_input_arg)
    f.write(string_to_file)
    f.write("\n=================\n\n")


if __name__ == '__main__':
    # init node
    rospy.init_node("sensor_placement_test_node")
    rospy.sleep(0.5)

    # setup client
    dyn_reconf_client = dynamic_reconfigure.client.Client("/sensor_placement")
    # get current config
    start_config = dyn_reconf_client.get_configuration(5.0)
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

    ## setup AoI and forbidden Area
    rospy.loginfo("Creating Area of interest")
    pub_AoI_sc()
    rospy.loginfo("Setup forbidden area")
    pub_forbidden_area_sc()

    # open file
    now = datetime.datetime.now()
    with open('result_'+now.strftime('%y%m%d')+'_'+now.strftime('%H%M%S')+'.txt','w') as f:
        string_to_file = "Results of test run at "+str(now)+"\n===================\n\n"
        f.write(string_to_file)
        f.write("Notes: \nThe config from the yaml file is always restored before updating the configuration.\n")
        f.write("This should help to see the differences from the start configuration. The start config is:\n")
        for key, value in start_config.iteritems():
            if not key == "groups":
                f.write(str(key)+": "+str(value)+"\n")
        f.write("\n\n\n\n")

        counter = 0
        # PSO with different number of sensors
        for num in number_of_sensors:
            counter += 1
            rospy.loginfo("Start PSO test num %d of %d", counter, len(number_of_sensors))
            update_config(dyn_reconf_client, start_config)
            update = {"number_of_sensors": str(num)}
            old_cfg, new_cfg = update_config(dyn_reconf_client, update)

            # call action
            goal.service_id = algo["PSO"]
            delta_t, result = call_action(client,goal)

            result_to_file(f, goal.service_id, update, result, delta_t)


        counter = 0
        # GreedyPSO with different number of sensors and particles
        for num in number_of_sensors:
            for particles in greedyPSO_particles:
                counter += 1
                rospy.loginfo("Start GreedyPSO test num %d of %d", counter, len(number_of_sensors)*len(greedyPSO_particles))
                update_config(dyn_reconf_client, start_config)
                update = {"number_of_sensors": str(num), "number_of_particles": str(particles)}
                old_cfg, new_cfg = update_config(dyn_reconf_client, update)

                # call action
                goal.service_id = algo["GreedyPSO"]
                delta_t, result = call_action(client,goal)

                result_to_file(f, goal.service_id, update, result, delta_t)

        counter = 0
        # Greedy with different number of sensors and different rasterization
        for num in number_of_sensors:
            for offset in gs_resolution:
                counter += 1
                rospy.loginfo("Start Greedy test num %d of %d", counter, len(number_of_sensors)*len(gs_resolution))
                update_config(dyn_reconf_client, start_config)
                update = {"number_of_sensors": str(num)}
                old_cfg, new_cfg = update_config(dyn_reconf_client, update)

                # call action
                goal.service_id = algo["GreedySearch"]
                # set offset
                goal.service_input_arg = offset
                delta_t, result = call_action(client,goal)
                
                result_to_file(f, goal.service_id, update, result, delta_t, service_input_arg=goal.service_input_arg)
                # to be sure, reset offset
                goal.service_input_arg = 0.0
                

        # Finished
        now = datetime.datetime.now()
        string_to_file = "\n\n===================\nFinished test runs at "+str(now)
        f.write(string_to_file)
