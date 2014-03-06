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
 * Author: Florian Mirus, email:Florian.Mirus@ipa.fhg.de
 *
 * Date of creation: April 2013
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

#ifndef SENSOR_PLACEMENT_H
#define SENSOR_PLACEMENT_H

// standard includes
#include <iostream>
#include <vector>
#include <math.h>
#include <time.h>
#include <stdlib.h>

// ros includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <ros/console.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/GetMap.h>
#include <actionlib/server/simple_action_server.h>

// external includes
#include <sensor_model.h>
#include <particle.h>
#include <seneka_utilities.h>
#include <greedySearch.h>
#include <clipper.hpp>
#include <seneka_sensor_placement/polygon_offset.h>
#include <seneka_sensor_placement/sensorPlacementAction.h>

using namespace std;
using namespace seneka_utilities;


class sensor_placement_node
{
private:

  // actual 2D-grid-map
  nav_msgs::OccupancyGrid map_;

  // bool variable to check if a map was already received
  bool map_received_;

  // bool variable to check if an AoI was already received
  bool AoI_received_;

  // bool variable to check if an forbidden area was already received
  bool fa_received_;

  // bool variable to check if an offset_polygon was already received
  bool polygon_offset_val_received_;

  // bool variable to check if the targets were already taken from the map
  bool targets_saved_;

  // actual area of interest to be covered by the sensor nodes
  geometry_msgs::PolygonStamped area_of_interest_;

  // polygon array for forbidden area
  std::vector<geometry_msgs::PolygonStamped> forbidden_area_vec_;

  // a vector for points of interest
  std::vector<geometry_msgs::Point> PoI_vec_;

  // number of sensors
  int sensor_num_;

  // maximal range of sensors
  double sensor_range_;

  // opening angles for sensors
  std::vector<double> open_angles_;

  // opening angles of FOV slice for greedy search
  std::vector<double> slice_open_angles_;

  // maximal allowed linear velocity for each sensor in particles
  double max_lin_vel_;

  // maximal allowed angular velocity for each sensor in particles
  double max_ang_vel_;

  // Lookup-Table for sensor raytracing.
  std::vector< std::vector<geometry_msgs::Point32> > lookup_table_;

  // number of particles for PSO
  int particle_num_;

  // maximal number of PSO iterations
  int iter_max_;

  // minimal coverage to stop PSO before reaching maximal number of PSO iterations
  double min_cov_;

  // maximal number of GreedyPSO iterations
  int iter_max_per_sensor_;

  // minimal coverage to stop GreedyPSO before reaching maximal number of iterations
  double min_sensor_cov_;

  // particle swarm
  vector<particle> particle_swarm_;

  // Greedy_PSO solution
  particle sol_particle_;

  // total number of targets covered by GreedyPSO
  unsigned int total_gPSO_covered_targets_num_;

  // vector storing the positions global best solution of the particle swarm
  particle global_best_;

  // best_priority_sum_ -b-
  int best_priority_sum_;

  // PSO actual best coverage
  double best_cov_;
  int global_best_multiple_coverage_;
  size_t best_particle_index_;

  // number of targets for PSO
  int target_num_;

  // target vectors (also holds occupation and coverage information)
  vector<target_info_fix> targets_with_info_fix_; //fix information
  vector<target_info_var> targets_with_info_var_; //variable information

  // vector of points (holds coordinates and potential target information)
  vector<point_info> point_info_vec_;

  // pool of points for Greedy placement of sensor (holds coordinates and max coverage information)
  vector<GS_point_info> GS_pool_;

  // Greedy search object
  greedySearch GS_solution;

  // offset value for offsetAoI function
  double clipper_offset_value_;

  // GS_target_offset_ parameter [in meters] for Greedy Search
  double GS_target_offset_;

  // PSO parameter constants
  double PSO_param_1_;
  double PSO_param_2_;
  double PSO_param_3_;

  // optimization result as nav_msgs::Path
  nav_msgs::Path PSO_result_;

  //variable to check if the action was completed successfully or not
  bool action_success_;


public:

  // constructor
  sensor_placement_node();

  // destrcutor
  ~sensor_placement_node();

  /* ----------------------------------- */
  /* --------- ROS Variables ----------- */
  /* ----------------------------------- */

  // create node handles
  ros::NodeHandle nh_, pnh_;

  // declaration of ros subscribers
  ros::Subscriber AoI_sub_;
  ros::Subscriber PoI_sub_;
  ros::Subscriber forbidden_area_sub_;

  // declaration of ros publishers
  ros::Publisher marker_array_pub_;
  ros::Publisher GS_targets_grid_pub_;
  ros::Publisher map_pub_, map_meta_pub_;
  ros::Publisher nav_path_pub_;
  ros::Publisher offset_AoI_pub_;
  ros::Publisher fa_marker_array_pub_;
//  ros::Publisher points_marker_array_pub_;

  // declaration of ros service clients
  ros::ServiceClient sc_get_map_;

  /* ----------------------------------- */
  /* ----------- functions ------------- */
  /* ----------------------------------- */

  // function to get the ROS parameters from yaml-file
  void getParams();

  // function to get an array of targets from the map and the area of interest specified as polygon
  bool getTargets();

  // function to get greedy search targets
  bool getGSTargets();

  // function to start map service and create look up tables
  void initializeCallback();

  // function to initialize PSO-Algorithm
  void initializePSO();

  // function to initialize Greedý-PSO-Algorithm
  void initializeGreedyPSO();

  // function to initialize GS-Algorithm
  void initializeGS();

  // function for the actual partcile-swarm-optimization
  void PSOptimize();

  // function for the  particle-swarm-optimization with Greedy optimization
  void GreedyPSOptimize();

  // function to run Greedy Search Algorithm
  void runGS();

  // function to get the current global best solution
  void getGlobalBest();

  // function to create an offsetted polygon from area of interest
  geometry_msgs::PolygonStamped offsetAoI(double offset);

  // function to return the visualization markers of a vector of polygons
  visualization_msgs::MarkerArray getPolygonVecVisualizationMarker(std::vector<geometry_msgs::PolygonStamped>, std::string);

  // function to return the visualization markers of a vector of points
 // visualization_msgs::MarkerArray getPointVecVisualizationMarker(std::vector<geometry_msgs::Point>, std::string);


  /* ----------------------------------- */
  /* --------- ROS Callbacks ----------- */
  /* ----------------------------------- */

  // callback function for the start PSO action
  bool startPSOCallback();

  // callback function for the start GreedyPSO action
  bool startGreedyPSOCallback();

  // callback function for the start GS action
  bool startGSCallback();

  // callback function for the start GS action with offset parameter
  bool startGSWithOffsetCallback();

  // callback function for clearing all forbidden areas
  bool clearFACallback();

  // callback function for the test action
  bool testServiceCallback();

  // callback functions
  void AoICB(const geometry_msgs::PolygonStamped::ConstPtr &AoI);
  void PoICB(const visualization_msgs::MarkerArray::ConstPtr &PoI);
  void forbiddenAreaCB(const geometry_msgs::PolygonStamped::ConstPtr &forbidden_areas);

  // goal callback function
  void executeGoalCB(const seneka_sensor_placement::sensorPlacementGoalConstPtr &goal);

  // function that gets executed immediately when the action is preempted
  void preemptCB();

  // function to cancel the goal if requested by action client (sensor_placement_interface) and returns true
  bool preemptRequested();


protected:

  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<seneka_sensor_placement::sensorPlacementAction> as_;

  // string to contain the action name
  std::string action_name_;

  // messages that are used to published feedback/result. NOTE: (not used yet)
  seneka_sensor_placement::sensorPlacementFeedback action_feedback_;
  seneka_sensor_placement::sensorPlacementResult action_result_;

};

#endif
