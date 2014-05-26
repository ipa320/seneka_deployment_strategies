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

import dynamic_reconfigure.client

from std_srvs.srv import Empty
from seneka_sensor_placement.srv import polygon_offset
        
def update_config(client, cfg_update):
    old_cfg = dyn_reconf_client.get_configuration(5.0)
    dyn_reconf_client.update_configuration(cfg_update)
    new_cfg = dyn_reconf_client.get_configuration(5.0)
    rospy.sleep(0.5)
    return old_cfg, new_cfg



if __name__ == '__main__':
    # init node
    rospy.init_node("sensor_placement_test_node")
    rospy.sleep(0.5)

    # setup client
    dyn_reconf_client = dynamic_reconfigure.client.Client("/sensor_placement")
    # get current config
    sensor_placement_config = dyn_reconf_client.get_configuration(5.0)
    rospy.loginfo("Dynamic reconfigure Client set up!")

    # setup services
    rospy.wait_for_service('StartPSO')
    PSO_sc = rospy.ServiceProxy('StartPSO', Empty)
    rospy.loginfo("PSO Service connected!")

    rospy.wait_for_service('StartGS')
    GS_sc = rospy.ServiceProxy('StartGS', Empty)
    rospy.loginfo("GS Service connected!")

    rospy.wait_for_service('StartGS_with_offset_polygon')
    GS_offset_sc = rospy.ServiceProxy('StartGS_with_offset_polygon', polygon_offset)
    rospy.loginfo("GS offset Service connected!")

    rospy.wait_for_service('StartGreedyPSO')
    GreedyPSO_sc = rospy.ServiceProxy('StartGreedyPSO', Empty)
    rospy.loginfo("GreedyPSO Service connected!")

    rospy.wait_for_service('CancelAction')
    Cancel_sc = rospy.ServiceProxy('CancelAction', Empty)
    rospy.loginfo("CancelAction Service connected!")

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

    # call services and run tests
    rospy.loginfo("Creating Area of interest")
    pub_AoI_sc()
    rospy.loginfo("Calling PSO with default config")
    PSO_sc()
    rospy.sleep(20.0)
    rospy.loginfo("Canceling Action")
    Cancel_sc()

    rospy.sleep(5.0)
    rospy.loginfo("Set new configuration")
    update = {"number_of_sensors": "7"}
    old_cfg, new_cfg = update_config(dyn_reconf_client, update)
    print old_cfg
    print ""
    print new_cfg

    rospy.loginfo("Calling PSO with default config")
    PSO_sc()
    rospy.sleep(20.0)
    rospy.loginfo("Canceling Action")
    Cancel_sc()
