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

#include <sensor_placement_node.h>

// constructor
sensor_placement_node::sensor_placement_node()
//initializer list
: nh_(std::string("")),
  as_(
     nh_,                                                                             //node handle
     std::string("sensorPlacementActionServer"),                                      //name the action server
     boost::bind(&sensor_placement_node::executeGoalCB, this, _1),                    //bind the goal callback function
     false),                                                                          //option to automatically spin a thread
     action_name_(std::string("sensorPlacementAction"))                               //name the action
{

  // create node handles
  pnh_ = ros::NodeHandle(std::string("~"));

  // ros subscribers
  AoI_sub_ = nh_.subscribe(std::string("in_AoI_poly"), 1,
                           &sensor_placement_node::AoICB, this);
  PoI_sub_ = nh_.subscribe(std::string("out_PoI_marker_array"), 1,                           //TODO: rename to in_PoI_MA
                           &sensor_placement_node::PoICB, this);
  forbidden_area_sub_ = nh_.subscribe(std::string("in_forbidden_area"), 1,
                                      &sensor_placement_node::forbiddenAreaCB, this);

  // ros publishers
  nav_path_pub_ = nh_.advertise<nav_msgs::Path>(std::string("out_path"),1,true);
  marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(std::string("out_marker_array"),1,true);
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(std::string("out_cropped_map"),1,true);
  map_meta_pub_ = nh_.advertise<nav_msgs::MapMetaData>(std::string("out_cropped_map_metadata"),1,true);
  offset_AoI_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>(std::string("offset_AoI"), 1,true);
  GS_targets_grid_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(std::string("GS_targets_grid"),1,true);
  fa_marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(std::string("fa_marker_array"),1,true);


  // ros service clients
  sc_get_map_ = nh_.serviceClient<nav_msgs::GetMap>(std::string("static_map"));

  // register the preempt callback function
  as_.registerPreemptCallback(boost::bind(&sensor_placement_node::preemptCB, this));

  // initialize action success variable
  action_success_ = true;

  // start the action server
  as_.start();

  // get parameters from parameter server if possible
  getParams();

  // set dynamic reconfigure server
  dyn_reconf_callback = boost::bind(&sensor_placement_node::configureCallback, this, _1, _2);
  dyn_reconf_server.setCallback(dyn_reconf_callback);

  // initialize best coverage
  best_cov_ = 0;

  // initialize global best multiple coverage
  global_best_multiple_coverage_ = 0;

  // initialize number of targets
  target_num_ = 0;

  // initialize total number of targets covered by GreedyPSO
  total_GreedyPSO_covered_targets_num_ = 0;

  // initialize best particle index
  best_particle_index_ = 0;

  // initialize best priority sum
  best_priority_sum_ = 0;

  // initialize other variables
  map_received_ = false;
  AoI_received_ = false;
  targets_saved_ = false;
  fa_received_ = false;
  polygon_offset_val_received_=false;

}

// destructor
sensor_placement_node::~sensor_placement_node(){}


// function to get the ROS parameters from yaml-file
void sensor_placement_node::getParams()
{
  int tmp = 0;
  if(!pnh_.hasParam(std::string("number_of_sensors")))
  {
    ROS_WARN("No parameter number_of_sensors on parameter server. Using default [5]");
  }
  pnh_.param(std::string("number_of_sensors"),tmp,5);
  sensor_num_ = (unsigned int) tmp;

  if(!pnh_.hasParam(std::string("max_sensor_range")))
  {
    ROS_WARN("No parameter max_sensor_range on parameter server. Using default [5.0 in m]");
  }
  pnh_.param(std::string("max_sensor_range"),sensor_range_,5.0);

  double open_angle_1, open_angle_2;

  if(!pnh_.hasParam(std::string("open_angle_1")))
  {
    ROS_WARN("No parameter open_angle_1 on parameter server. Using default [1.5708 in rad]");
  }
  pnh_.param(std::string("open_angle_1"),open_angle_1,1.5708);

  open_angles_.push_back(open_angle_1);

  if(!pnh_.hasParam(std::string("open_angle_2")))
  {
    ROS_WARN("No parameter open_angle_2 on parameter server. Using default [0.0 in rad]");
  }
  pnh_.param(std::string("open_angle_2"),open_angle_2,0.0);

  open_angles_.push_back(open_angle_2);

  if(!pnh_.hasParam(std::string("max_linear_sensor_velocity")))
  {
    ROS_WARN("No parameter max_linear_sensor_velocity on parameter server. Using default [1.0]");
  }
  pnh_.param(std::string("max_linear_sensor_velocity"),max_lin_vel_,1.0);

  if(!pnh_.hasParam(std::string("max_angular_sensor_velocity")))
  {
    ROS_WARN("No parameter max_angular_sensor_velocity on parameter server. Using default [0.5236]");
  }
  pnh_.param(std::string("max_angular_sensor_velocity"),max_ang_vel_,0.5236);

  if(!pnh_.hasParam(std::string("number_of_particles")))
  {
    ROS_WARN("No parameter number_of_particles on parameter server. Using default [20]");
  }
  pnh_.param(std::string("number_of_particles"),particle_num_,20);

  if(!pnh_.hasParam(("max_num_iterations")))
  {
    ROS_WARN("No parameter max_num_iterations on parameter server. Using default [400]");
  }
  pnh_.param(std::string("max_num_iterations"),iter_max_,400);

  if(!pnh_.hasParam(std::string("max_num_iterations_per_sensor")))
  {
    ROS_WARN("No parameter max_num_iterations_per_sensor on parameter server. Using default [30]");
  }
  pnh_.param(std::string("max_num_iterations_per_sensor"),iter_max_per_sensor_,30);

  if(!pnh_.hasParam(std::string("min_coverage_to_stop")))
  {
    ROS_WARN("No parameter min_coverage_to_stop on parameter server. Using default [0.95]");
  }
  pnh_.param(std::string("min_coverage_to_stop"),min_cov_,0.95);

  if(!pnh_.hasParam(std::string("min_sensor_coverage_to_stop")))
  {
    ROS_WARN("No parameter min_sensor_coverage_to_stop on parameter server. Using default [0.08]");
  }
  pnh_.param(std::string("min_sensor_coverage_to_stop"),min_sensor_cov_, 0.08);

  // get PSO configuration parameters
  if(!pnh_.hasParam(std::string("c1")))
  {
    ROS_WARN("No parameter c1 on parameter server. Using default [0.729]");
  }
  pnh_.param(std::string("c1"),PSO_param_1_,0.729);

  if(!pnh_.hasParam(std::string("c2")))
  {
    ROS_WARN("No parameter c2 on parameter server. Using default [1.49445]");
  }
  pnh_.param(std::string("c2"),PSO_param_2_,1.49445);

  if(!pnh_.hasParam(std::string("c3")))
  {
    ROS_WARN("No parameter c3 on parameter server. Using default [1.49445]");
  }
  pnh_.param(std::string("c3"),PSO_param_3_,1.49445);

  // get Greedy Search algorithm parameters
  double slice_open_angle_1, slice_open_angle_2;

  if(!pnh_.hasParam(std::string("slice_open_angle_1")))
  {
    ROS_WARN("No parameter slice_open_angle_1 on parameter server. Using default [1.5708 in rad]");
  }
  pnh_.param(std::string("slice_open_angle_1"),slice_open_angle_1,1.5708);

  slice_open_angles_.push_back(slice_open_angle_1);

  if(!pnh_.hasParam(std::string("slice_open_angle_2")))
  {
    ROS_WARN("No parameter slice_open_angle_2 on parameter server. Using default [0.0 in rad]");
  }
  pnh_.param(std::string("slice_open_angle_2"),slice_open_angle_2,0.0);

  slice_open_angles_.push_back(slice_open_angle_2);

  if(!pnh_.hasParam(std::string("GS_target_offset")))
  {
    ROS_WARN("No parameter GS_target_offset on parameter server. Using default [5.0 in m]");
  }
  pnh_.param(std::string("GS_target_offset"),GS_target_offset_, 5.0);
}

// function to get the ROS parameters from dynamic reconfigure
void sensor_placement_node::configureCallback(seneka_sensor_placement::seneka_sensor_placementConfig &config, uint32_t level)
{
  if (as_.isActive())
    ROS_WARN("Cannot set new parameters while optimization is running!");
  else
  {
    sensor_num_ = config.number_of_sensors;
    sensor_range_ = config.max_sensor_range;
    open_angles_.at(0) = config.open_angle_1;
    open_angles_.at(1) = config.open_angle_2;
    max_lin_vel_ = config.max_linear_sensor_velocity;
    max_ang_vel_ = config.max_angular_sensor_velocity;
    particle_num_ = config.number_of_particles;
    iter_max_ = config.max_num_iterations;
    min_cov_ = config.min_coverage_to_stop;
    PSO_param_1_ = config.c1;
    PSO_param_2_ = config.c2;
    PSO_param_3_ = config.c3;
    iter_max_per_sensor_ = config.max_num_iterations_per_sensor;
    min_sensor_cov_ = config.min_coverage_to_stop_per_sensor;
    slice_open_angles_.at(0) = config.slice_open_angle_1;
    slice_open_angles_.at(1) = config.slice_open_angle_2;
    GS_target_offset_ = config.GS_target_offset;
  }
}


// function to cancel the goal if requested by action client (sensor_placement_interface) and returns true if preepmtion is requested
bool sensor_placement_node::preemptRequested()
{
  // check if action preempt has been requested by the client
  if (as_.isPreemptRequested() || !ros::ok())
  {
    // if the action is still in active state, set the action state to preempted now
    if (as_.isActive())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
    }
    //set action success to be false
    action_success_ = false;
    return true;
  }
  return false;
}


// function that gets executed immediately when the action is preempted
void sensor_placement_node::preemptCB()
{
  //can be used later to do anything before the action gets aborted
}

// goal callback function
void sensor_placement_node::executeGoalCB(const seneka_sensor_placement::sensorPlacementGoalConstPtr &goal)
{
  // helper variables
  ros::Rate r(1);

  //reset default action_success_ status to be true
  action_success_ = true;

  switch (goal->service_id)
  {
    case 1:
    {
      ROS_INFO("Starting 'PSO' action");
      startPSOCallback();
      break;
    }

    case 2:
    {
      ROS_INFO("Starting 'GreedyPSO' action");
      startGreedyPSOCallback();
      break;
    }

    case 3:
    {
      ROS_INFO("Starting 'GreedySearch' action");
      startGSCallback();
      break;
    }

    case 4:
    {
      ROS_INFO("Starting 'GreedySearch_with_offset_polygon' action");
      // save offset value received
      clipper_offset_value_ = goal->service_input_arg;
      polygon_offset_val_received_=true;
      startGSWithOffsetCallback();
      break;
    }

    case 5:
    {
      ROS_INFO("Performing 'Clear_forbidden_areas' action");
      clearFACallback();
      break;
    }


    case 6:
    {
      ROS_INFO("Starting 'Test' action");
      testServiceCallback();
      break;
    }

    default:
    {
      ROS_ERROR("Invlaid service_id");
      action_success_ = false;
    }
  }

  if(action_success_)
  {
    seneka_sensor_placement::sensorPlacementResult result;
    result.coverage = best_cov_;
    ROS_INFO("Action Succeeded with coverage of %f", best_cov_);
    // set the action state to succeeded
    as_.setSucceeded(result);
  }
  else
    ROS_INFO("Action was not completed");
}



bool sensor_placement_node::getTargets()
{
  // initialize result
  bool result = false;
  // intialize local variable
  size_t num_of_fa = forbidden_area_vec_.size();

  if(map_received_ == true)
  {
    // only if we received a map, we can get targets
    target_info_fix dummy_target_info_fix;
    targets_with_info_fix_.assign(map_.info.width * map_.info.height, dummy_target_info_fix);
    target_info_var dummy_target_info_var;
    targets_with_info_var_.assign(map_.info.width * map_.info.height, dummy_target_info_var);
    dummy_target_info_var.covered_by_sensor.assign(sensor_num_, false);

    if(AoI_received_ == false)
    {
      for(unsigned int i = 0; i < map_.info.width; i++)
      {
        for(unsigned int j = 0; j < map_.info.height; j++)
        {
          // calculate world coordinates from map coordinates of given target
          geometry_msgs::Pose2D world_Coord;
          world_Coord.x = mapToWorldX(i, map_);
          world_Coord.y = mapToWorldY(j, map_);
          world_Coord.theta = 0;

          //fix information
          dummy_target_info_fix.world_pos.x = mapToWorldX(i, map_);
          dummy_target_info_fix.world_pos.y = mapToWorldY(j, map_);
          dummy_target_info_fix.world_pos.z = 0;

          dummy_target_info_fix.forbidden = false;                          //all targets are allowed unless found in forbidden area
          dummy_target_info_fix.occupied = true;
          dummy_target_info_fix.potential_target = -1;
          dummy_target_info_fix.priority = 0;                                //all targets other than PoI have priority = 0

          // check if current target is a point of interest
          for (size_t ii=0; ii<PoI_vec_.size(); ii++)
          {
            if((world_Coord.x == PoI_vec_.at(ii).x) && (world_Coord.y == PoI_vec_.at(ii).y))
            {
              // found a point of interest, set priority
              dummy_target_info_fix.priority = 100;
              break;
            }
          }

          //variable information
          dummy_target_info_var.covered = false;
          dummy_target_info_var.multiple_covered = false;
          dummy_target_info_var.no_reset = false;

          if(map_.data.at( j * map_.info.width + i) == 0)
          {
            //check all forbidden areas
            for (size_t k=0; k<num_of_fa; k++)
            {
              if((pointInPolygon(world_Coord, forbidden_area_vec_.at(k).polygon) >= 1) && (fa_received_==true))
              {
                // the given position is on the forbidden area
                dummy_target_info_fix.forbidden = true;
                break;
              }
            }
            dummy_target_info_fix.occupied = false;
            dummy_target_info_fix.potential_target = 1;

            target_num_++;
          }
        }
      }
      result = true;
    }
    else
    {
      // if area of interest polygon was specified, we consider the non-occupied grid cells within the polygon as targets
      for(unsigned int i = 0; i < map_.info.width; i++)
      {
        for(unsigned int j = 0; j < map_.info.height; j++)
        {
          // calculate world coordinates from map coordinates of given target
          geometry_msgs::Pose2D world_Coord;
          world_Coord.x = mapToWorldX(i, map_);
          world_Coord.y = mapToWorldY(j, map_);
          world_Coord.theta = 0;


          //fix information
          dummy_target_info_fix.world_pos.x = world_Coord.x;
          dummy_target_info_fix.world_pos.y = world_Coord.y;
          dummy_target_info_fix.world_pos.z = 0;

          dummy_target_info_fix.forbidden = false;                                         //all targets are allowed unless found in forbidden area
          dummy_target_info_fix.occupied = true;
          dummy_target_info_fix.potential_target = -1;
          dummy_target_info_fix.map_data = map_.data.at( j * map_.info.width + i);
          dummy_target_info_fix.priority = 0;                                              //all targets other than PoI have priority = 0

          // check if current target is a point of interest
          for (size_t ii=0; ii<PoI_vec_.size(); ii++)
          {
            if((world_Coord.x == PoI_vec_.at(ii).x) && (world_Coord.y == PoI_vec_.at(ii).y))
            {
              // found a point of interest, set priority
              dummy_target_info_fix.priority = 100;
              break;
            }
          }

          //variable information
          dummy_target_info_var.covered = false;
          dummy_target_info_var.multiple_covered = false;
          dummy_target_info_var.no_reset = false;

          // the given position lies withhin the polygon
          if(pointInPolygon(world_Coord, area_of_interest_.polygon) == 2)
          {
            //check all forbidden areas
            for (size_t k=0; k<num_of_fa; k++)
            {
              if((pointInPolygon(world_Coord, forbidden_area_vec_.at(k).polygon) >= 1) && (fa_received_==true))
              {
                // the given position is on the forbidden area
                dummy_target_info_fix.forbidden = true;
                break;
              }
            }
            dummy_target_info_fix.potential_target = 1;

            if(map_.data.at( j * map_.info.width + i) == 0)
            {
              target_num_++;
              dummy_target_info_fix.occupied = false;
            }
          }
          // the given position lies on the perimeter
          else if( pointInPolygon(world_Coord, area_of_interest_.polygon) == 1)
          {
            //check all forbidden areas
            for (size_t k=0; k<num_of_fa; k++)
            {
              if((pointInPolygon(world_Coord, forbidden_area_vec_.at(k).polygon) >= 1) && (fa_received_==true))
              {
                // the given position is on the forbidden area
                dummy_target_info_fix.forbidden = true;
                break;
              }
            }
            dummy_target_info_fix.potential_target = 0;

            if(map_.data.at( j * map_.info.width + i) == 0)
            {
              dummy_target_info_fix.occupied = false;
            }
          }
          // save the target information
          targets_with_info_var_.at(j * map_.info.width + i) = dummy_target_info_var;
          targets_with_info_fix_.at(j * map_.info.width + i) = dummy_target_info_fix;
        }
      }
      result = true;
    }
  }
  else
  {
    ROS_WARN("No map received! Not able to propose sensor positions.");
  }
  return result;
}


// get a pool of points where the greedySearch algorithm looks for optimal coverage
bool sensor_placement_node::getGSTargets()
{
  // initialize result
  bool result = false;
  // intialize local variables
  size_t num_of_fa = forbidden_area_vec_.size();
  // initialize flag to identify that a given point is in forbidden area
  bool fa_flag = false;
  // intialize cell offset (number of cells to be skipped)
  unsigned int cell_offset = (unsigned int) round(GS_target_offset_/map_.info.resolution);
  // calculate priority value for PoI
  unsigned int priority_value = calculateMaxSensorCoverage(sensor_range_,open_angles_);

  if(cell_offset==0)
  {
    cell_offset=1;
    ROS_WARN("evaluated cell_offset = 0, forcing cell_offset = 1");
  }

  if(map_received_ == true)
  {
    //only if we received a map, we can get targets
    point_info dummy_point_info;
    dummy_point_info.occupied = false;
    dummy_point_info.covered = false;
    point_info_vec_.assign(map_.info.width * map_.info.height, dummy_point_info);
    //create dummy GS_point to save information in the pool
    GS_point dummy_GS_point;

    if(AoI_received_ == false)
    {
      for(unsigned int i = 0; i < map_.info.width; i++)
      {
        for(unsigned int j = 0; j < map_.info.height; j++)
        {
          //calculate world coordinates from map coordinates of given target
          geometry_msgs::Pose2D world_Coord;
          world_Coord.x = mapToWorldX(i, map_);
          world_Coord.y = mapToWorldY(j, map_);
          world_Coord.theta = 0;

          dummy_point_info.occupied = true;
          dummy_point_info.potential_target = -1;
          dummy_point_info.priority = 0;                              //all points other than PoI have priority = 0

          // check if current point is a point of interest
          for (size_t ii=0; ii<PoI_vec_.size(); ii++)
          {
            if((world_Coord.x == PoI_vec_.at(ii).x) && (world_Coord.y == PoI_vec_.at(ii).y))
            {
              // found a point of interest, set priority
              ROS_INFO_STREAM("Setting " << priority_value << " as priority value for PoI at (" << world_Coord.x << "," << world_Coord.y << ")");
              dummy_point_info.priority = priority_value;
              break;
            }
          }

          //check if the given point is occupied or not; mark non-occupied ones as potential targets
          if(map_.data.at( j * map_.info.width + i) == 0)
          {
            dummy_point_info.occupied = false;
            dummy_point_info.potential_target = 1;
            target_num_++;
            //check if the given point lies on the grid
            if ((i%cell_offset==0) && (j%cell_offset==0))
            {
              if (fa_received_==true)
              {
                //check all forbidden areas
                for (size_t k=0; k<num_of_fa; k++)
                {
                  if(pointInPolygon(world_Coord, forbidden_area_vec_.at(k).polygon) >= 1)
                  {
                    fa_flag=true;
                    break;
                  }
                }
                //allow only targets outside all forbidden areas to be saved as GS_target
                if(fa_flag==false)
                {
                  //given point is not occupied AND on the grid AND not in any forbidden area. So, save it as GS_target
                  dummy_GS_point.x=i;
                  dummy_GS_point.y=j;
                  GS_pool_.push_back(dummy_GS_point);
                  fa_flag=false;
                }
                else
                {
                  //skip the given point and make the flag false again
                  fa_flag=false;
                }
              }
              else
              {
                //given point is not occupied AND on the grid. So, save it as GS_target
                dummy_GS_point.x=i;
                dummy_GS_point.y=j;
                GS_pool_.push_back(dummy_GS_point);
              }
            }
          }
          // saving point information
          point_info_vec_.at(j * map_.info.width + i) = dummy_point_info;
        }
      }
      result = true;
    }
    else
    {
      //AoI received, check if polygon offset value is received
      if(polygon_offset_val_received_==true)
      {
        //****************get offset polygon****************
        geometry_msgs::PolygonStamped offset_AoI;
        double  perimeter_min = 3*map_.info.resolution;     // define minimum perimiter in meters
        double offset_val = clipper_offset_value_;          // load offset value

        // set a lower bound on offset value according to perimeter
        if (offset_val < perimeter_min)
          offset_val = perimeter_min;

        // now get an offsetted polygon from area of interest
        offset_AoI = offsetAoI(offset_val);

        if (!offset_AoI.polygon.points.empty())
        {
          //publish offset_AoI
          offset_AoI.header.frame_id = std::string("/map");
          offset_AoI_pub_.publish(offset_AoI);
        }
        else
        {
          ROS_ERROR("No offset polygon returned");
          return false;
        }
        //****************now get targets******************
        for(unsigned int i = 0; i < map_.info.width; i++)
        {
          for(unsigned int j = 0; j < map_.info.height; j++)
          {
            // calculate world coordinates from map coordinates of given target
            geometry_msgs::Pose2D world_Coord;
            world_Coord.x = mapToWorldX(i, map_);
            world_Coord.y = mapToWorldY(j, map_);
            world_Coord.theta = 0;

            dummy_point_info.occupied = true;
            dummy_point_info.potential_target = -1;
            dummy_point_info.priority = 0;                              //all points other than PoI have priority = 0

            // check if current point is a point of interest
            for (size_t ii=0; ii<PoI_vec_.size(); ii++)
            {
              if((world_Coord.x == PoI_vec_.at(ii).x) && (world_Coord.y == PoI_vec_.at(ii).y))
              {
                // found a point of interest, set priority
                ROS_INFO_STREAM("Setting " << priority_value << " as priority value for PoI at (" << world_Coord.x << "," << world_Coord.y << ")");
                dummy_point_info.priority = priority_value;
                break;
              }
            }

            // the given point lies withhin the polygon
            if(pointInPolygon(world_Coord, area_of_interest_.polygon) == 2)
            {
              dummy_point_info.potential_target = 1;

              if(map_.data.at( j * map_.info.width + i) == 0)
              {
                dummy_point_info.occupied = false;
                target_num_++;
                //check if the given point lies on the grid
                if ((i%cell_offset==0) && (j%cell_offset==0))
                {
                  if(pointInPolygon(world_Coord, offset_AoI.polygon) == 0)
                  {
                    if (fa_received_ == true)
                    {
                      //check all forbidden areas
                      for (size_t k=0; k<num_of_fa; k++)
                      {
                        if(pointInPolygon(world_Coord, forbidden_area_vec_.at(k).polygon) >= 1)
                        {
                          fa_flag=true;
                          break;
                        }
                      }
                       //allow only targets outside all forbidden areas to be saved as GS_target
                      if(fa_flag==false)
                      {
                        //given point is not occupied AND on the grid AND outside offset_AoI, all forbidden areas. So, save it as GS_target
                        dummy_GS_point.x=i;
                        dummy_GS_point.y=j;
                        GS_pool_.push_back(dummy_GS_point);

                      }
                      else
                      {
                        //skip the given point and make the flag false again
                        fa_flag=false;
                      }
                    }
                    else
                    {
                      //given point is not occupied AND on the grid AND outside offset_AoI. So, save it as GS_target
                      dummy_GS_point.x=i;
                      dummy_GS_point.y=j;
                      GS_pool_.push_back(dummy_GS_point);
                    }
                  }
                }
              }
            }
            //the given point lies on the perimeter
            else if(pointInPolygon(world_Coord, area_of_interest_.polygon) == 1)
            {
              dummy_point_info.potential_target = 0;
              //check if given point is occupied or not
              if(map_.data.at( j * map_.info.width + i) == 0)
              {
                dummy_point_info.occupied = false;
                //check if the given point lies on the grid
                if ((i%cell_offset==0) && (j%cell_offset==0))
                {
                  if(pointInPolygon(world_Coord, offset_AoI.polygon) == 0)
                  {
                    if (fa_received_ == true)
                    {
                      //check all forbidden areas
                      for (size_t k=0; k<num_of_fa; k++)
                      {
                        if(pointInPolygon(world_Coord, forbidden_area_vec_.at(k).polygon) >= 1)
                        {
                          fa_flag=true;
                          break;
                        }
                      }
                      //allow only targets outside all forbidden areas to be saved as GS_target
                      if(fa_flag==false)
                      {
                        //given point is not occupied AND on the grid AND not in any forbidden area. So, save it as GS_target
                        dummy_GS_point.x=i;
                        dummy_GS_point.y=j;
                        GS_pool_.push_back(dummy_GS_point);
                      }
                      else
                      {
                        //skip the given point and make the flag false again
                        fa_flag=false;
                      }
                    }

                    else
                    {
                      //given point is not occupied AND on the grid. So, save it as GS_target
                      dummy_GS_point.x=i;
                      dummy_GS_point.y=j;
                      GS_pool_.push_back(dummy_GS_point);
                    }
                  }
                }
              }
            }
            // save information
            point_info_vec_.at(j * map_.info.width + i) = dummy_point_info;
          }
        }
        result = true;
      }
      else
      {
        //polygon_offset_val_received_ not recieved, get targets on whole AoI
        for(unsigned int i = 0; i < map_.info.width; i++)
        {
          for(unsigned int j = 0; j < map_.info.height; j++)
          {
            // calculate world coordinates from map coordinates of given target
            geometry_msgs::Pose2D world_Coord;
            world_Coord.x = mapToWorldX(i, map_);
            world_Coord.y = mapToWorldY(j, map_);
            world_Coord.theta = 0;

            dummy_point_info.occupied = true;
            dummy_point_info.potential_target = -1;
            dummy_point_info.priority = 0;                              //all points other than PoI have priority = 0

            // check if current point is a point of interest
            for (size_t ii=0; ii<PoI_vec_.size(); ii++)
            {
              if((world_Coord.x == PoI_vec_.at(ii).x) && (world_Coord.y == PoI_vec_.at(ii).y))
              {
                // found a point of interest, set priority
                ROS_INFO_STREAM("Setting " << priority_value << " as priority value for PoI at (" << world_Coord.x << "," << world_Coord.y << ")");
                dummy_point_info.priority = priority_value;
                break;
              }
            }

            // the given point lies withhin the polygon
            if(pointInPolygon(world_Coord, area_of_interest_.polygon) == 2)
            {
              dummy_point_info.potential_target = 1;
              // check if given point is occupied or not
              if(map_.data.at( j * map_.info.width + i) == 0)
              {
                dummy_point_info.occupied = false;
                target_num_++;
                //check if the given point lies on the grid
                if ((i%cell_offset==0) && (j%cell_offset==0))
                {
                  if (fa_received_ == true)
                  {
                    //check all forbidden areas
                    for (size_t k=0; k<num_of_fa; k++)
                    {
                      if(pointInPolygon(world_Coord, forbidden_area_vec_.at(k).polygon) >= 1)
                      {
                        fa_flag=true;
                        break;
                      }
                    }
                    //allow only targets outside all forbidden areas to be saved as GS_target
                    if(fa_flag==false)
                    {
                      //given point is not occupied AND on the grid AND not in any forbidden area. So, save it as GS_target
                      dummy_GS_point.x=i;
                      dummy_GS_point.y=j;
                      GS_pool_.push_back(dummy_GS_point);
                    }
                    else
                    {
                      //skip the given point and make the flag false again
                      fa_flag=false;
                    }
                  }
                  else
                  {
                    //given point is not occupied AND on the grid. So, save it as GS_target
                    dummy_GS_point.x=i;
                    dummy_GS_point.y=j;
                    GS_pool_.push_back(dummy_GS_point);
                  }
                }
              }
            }
            // the given point lies on the perimeter
            else if(pointInPolygon(world_Coord, area_of_interest_.polygon) == 1)
            {
              dummy_point_info.potential_target = 0;
              // check if given point is occupied or not
              if(map_.data.at( j * map_.info.width + i) == 0)
              {
                dummy_point_info.occupied = false;
                //check if the given point lies on the grid
                if ((i%cell_offset==0) && (j%cell_offset==0))
                {
                  if (fa_received_ == true)
                  {
                    //check all forbidden areas
                    for (size_t k=0; k<num_of_fa; k++)
                    {
                      if(pointInPolygon(world_Coord, forbidden_area_vec_.at(k).polygon) >= 1)
                      {
                        fa_flag=true;
                        break;
                      }
                    }
                    //allow only targets outside all forbidden areas to be saved as GS_target
                    if(fa_flag==false)
                    {
                      //given point is not occupied AND on the grid AND not in any forbidden area. So, save it as GS_target
                      dummy_GS_point.x=i;
                      dummy_GS_point.y=j;
                      GS_pool_.push_back(dummy_GS_point);
                    }
                    else
                    {
                      //skip the given point and make the flag false again
                      fa_flag=false;
                    }
                  }
                  else
                  {
                    //given point is not occupied AND on the grid. So, save it as GS_target
                    dummy_GS_point.x=i;
                    dummy_GS_point.y=j;
                    GS_pool_.push_back(dummy_GS_point);
                  }
                }
              }
            }
            // save information
            point_info_vec_.at(j * map_.info.width + i) = dummy_point_info;
          }
        }
        result = true;
      }
    }
  }
  else
  {
    ROS_WARN("No map received! Not able to propose sensor positions.");
  }
  return result;
}

// return an area of interest polygon which is offsetted according to the offset input
geometry_msgs::PolygonStamped sensor_placement_node::offsetAoI(double offset)
{
  //intializations
  ClipperLib::Polygon cl_AoI;
  ClipperLib::IntPoint cl_point;
  ClipperLib::Polygons in_polys;
  ClipperLib::Polygons out_polys;
  ClipperLib::JoinType jointype = ClipperLib::jtMiter;
  double miterLimit = 0.0;
  geometry_msgs::Point32 p;
  geometry_msgs::PolygonStamped offset_AoI;

  offset = doubleToInt(offset);

  // check if offset input is valid for deflating the area of interest
  if(offset>0)
  {
    //save AoI as a clipper library polygon
    for (size_t i=0; i<area_of_interest_.polygon.points.size(); i++)
    {
      cl_point.X = doubleToInt(area_of_interest_.polygon.points.at(i).x);
      cl_point.Y = doubleToInt(area_of_interest_.polygon.points.at(i).y);
      cl_AoI.push_back(cl_point);
    }

    //save the AoI
    in_polys.push_back(cl_AoI);

    //apply offset function to get an offsetted polygon in out_polys
    ClipperLib::OffsetPolygons(in_polys, out_polys, -1*offset, jointype, miterLimit);

    //check if the offset function returned only one polygon (if more, then in_polys is set incorrectly)
    if(out_polys.size()==1)
    {
      //save the offsetted polygon as geometry_msgs::PolygonStamped type
      for (size_t i=0; i<out_polys.at(0).size(); i++)
      {
        p.x = intToDouble(out_polys.at(0).at(i).X);
        p.y = intToDouble(out_polys.at(0).at(i).Y);
        p.z = 0;
        offset_AoI.polygon.points.push_back(p);
      }
    }
    else
    {
      ROS_ERROR("wrong output from ClipperLib::OffsetPolygons function");
    }

    // show output
    for (unsigned int i=0; i<offset_AoI.polygon.points.size(); i++)
    {
      ROS_INFO_STREAM("offset_AoI point " << i << " (" << offset_AoI.polygon.points.at(i).x << "," << offset_AoI.polygon.points.at(i).y << ")" ) ;
    }
  }
  // offset input is invalid
  else
    ROS_ERROR("wrong offset input! Enter a positive offset value for deflated polygon");

  return offset_AoI;
}

// function to initialize PSO-Algorithm
void sensor_placement_node::initializePSO()
{
  // initialize pointer to dummy sensor_model
  FOV_2D_model dummy_2D_model;
  dummy_2D_model.setMaxVelocity(max_lin_vel_, max_lin_vel_, max_lin_vel_, max_ang_vel_, max_ang_vel_, max_ang_vel_);

  // initialize dummy particle
  particle dummy_particle = particle(sensor_num_, target_num_, dummy_2D_model);

  dummy_particle.setMap(map_);
  dummy_particle.setAreaOfInterest(area_of_interest_);
  dummy_particle.setForbiddenAreaVec(forbidden_area_vec_);
  dummy_particle.setOpenAngles(open_angles_);
  dummy_particle.setRange(sensor_range_);
  dummy_particle.setTargetsWithInfoVar(targets_with_info_var_);
  dummy_particle.setTargetsWithInfoFix(targets_with_info_fix_, target_num_);
  dummy_particle.setLookupTable(& lookup_table_);

  // initialize particle swarm with given number of particles containing given number of sensors
  particle_swarm_.assign(particle_num_,dummy_particle);
  // initialize the global best solution
  global_best_ = dummy_particle;

  double actual_coverage = 0;

  // initialize sensors randomly on perimeter for each particle with random velocities
  if(AoI_received_)
  {
    for(size_t i = 0; i < particle_swarm_.size(); i++)
    {
      // initialize sensor poses randomly on perimeter
      particle_swarm_.at(i).initializeSensorsOnPerimeter();
      // initialize sensor velocities randomly
      particle_swarm_.at(i).initializeRandomSensorVelocities();
      // get calculated coverage
      actual_coverage = particle_swarm_.at(i).getActualCoverage();
      // check if the actual coverage is a new global best
      if(actual_coverage > best_cov_)
      {
        best_cov_ = actual_coverage;
        global_best_ = particle_swarm_.at(i);
      }
    }
    // after the initialization step we're looking for a new global best solution
    getGlobalBest();

    // publish the actual global best visualization
    marker_array_pub_.publish(global_best_.getVisualizationMarkers());
  }
}

// function to initialize PSO-Algorithm
void sensor_placement_node::initializeGreedyPSO()
{
  // initialize pointer to dummy sensor_model
  FOV_2D_model dummy_2D_model;
  dummy_2D_model.setMaxVelocity(max_lin_vel_, max_lin_vel_, max_lin_vel_, max_ang_vel_, max_ang_vel_, max_ang_vel_);

  // initialize dummy particle for GreedyPSO having only one sensor
  particle dummy_particle = particle(1, target_num_, dummy_2D_model);

  dummy_particle.setMap(map_);
  dummy_particle.setAreaOfInterest(area_of_interest_);
  dummy_particle.setForbiddenAreaVec(forbidden_area_vec_);
  dummy_particle.setOpenAngles(open_angles_);
  dummy_particle.setRange(sensor_range_);
  dummy_particle.setTargetsWithInfoVar(targets_with_info_var_);
  dummy_particle.setTargetsWithInfoFix(targets_with_info_fix_, target_num_);
  dummy_particle.setLookupTable(& lookup_table_);

  // initialize particle swarm with given number of particles, each containing only one sensor
  particle_swarm_.assign(particle_num_,dummy_particle);
  // initialize the global best solution
  global_best_ = dummy_particle;

  // initialize dummy solution particle with given number of sensors
  particle dummy_sol_particle = particle(sensor_num_, target_num_, dummy_2D_model);

  dummy_sol_particle.setMap(map_);
  dummy_sol_particle.setAreaOfInterest(area_of_interest_);
  dummy_sol_particle.setForbiddenAreaVec(forbidden_area_vec_);
  dummy_sol_particle.setOpenAngles(open_angles_);
  dummy_sol_particle.setRange(sensor_range_);
  dummy_sol_particle.setTargetsWithInfoVar(targets_with_info_var_);
  dummy_sol_particle.setTargetsWithInfoFix(targets_with_info_fix_, target_num_);
  dummy_sol_particle.setLookupTable(& lookup_table_);

  // set solution particle
  sol_particle_ = dummy_sol_particle;
}

// function to initialize GS-Algorithm
void sensor_placement_node::initializeGS()
{
  // initialize pointer to dummy sensor_model
  FOV_2D_model dummy_GS_2D_model;
  dummy_GS_2D_model.setMaxVelocity(max_lin_vel_, max_lin_vel_, max_lin_vel_, max_ang_vel_, max_ang_vel_, max_ang_vel_);

  // initialize dummy greedySearch object
  greedySearch gs_dummy = greedySearch(sensor_num_, target_num_, dummy_GS_2D_model);

  GS_solution = gs_dummy;
  GS_solution.setMap(map_);
  GS_solution.setAreaOfInterest(area_of_interest_);
  GS_solution.setOpenAngles(open_angles_);
  GS_solution.setSliceOpenAngles(slice_open_angles_);
  GS_solution.setRange(sensor_range_);
  GS_solution.setPointInfoVec(point_info_vec_, target_num_);
  GS_solution.setGSpool(GS_pool_);
  GS_solution.setLookupTable(&lookup_table_);
  GS_solution.setActionServer(&as_);

  if (!GS_pool_.empty())
  {
    //publish the GS_targarts grid
    GS_targets_grid_pub_.publish(GS_solution.getGridVisualizationMarker());
  }
  else
    ROS_ERROR("No targets in GS_pool_");

}

// function for the actual particle-swarm-optimization
void sensor_placement_node::PSOptimize()
{

  // clock_t t_start;
  // clock_t t_end;
  // double  t_diff;

  // PSO-iterator
  int iter = 0;
  std::vector<geometry_msgs::Pose> global_pose;

  // iteration step
  // continue calculation as long as there are iteration steps left and actual best coverage is
  // lower than mininmal coverage to stop
  // and goal cancellation is not requested by action client
  while(iter < iter_max_ && best_cov_ < min_cov_ && !preemptRequested())
  {
    global_pose = global_best_.getSolutionPositions();
    // update each particle in vector
    #pragma omp parallel for
      for(size_t i=0; i < particle_swarm_.size(); i++)
      {
        //t_start = clock();
        particle_swarm_.at(i).resetTargetsWithInfoVar();    //priority sum is also resetted here
        //t_end = clock();
        //t_diff = (double)(t_end - t_start) / (double)CLOCKS_PER_SEC;
        //ROS_INFO( "reset: %10.10f \n", t_diff);

        // now we're ready to update the particle
        //t_start = clock();
        particle_swarm_.at(i).updateParticle(global_pose, PSO_param_1_, PSO_param_2_, PSO_param_3_);
        //t_end = clock();
        //t_diff = (double)(t_end - t_start) / (double)CLOCKS_PER_SEC;
        //ROS_INFO( "updateParticle: %10.10f \n", t_diff);
      }

    // after the update step we're looking for a new global best solution
    getGlobalBest();

    // publish the actual global best visualization
    marker_array_pub_.publish(global_best_.getVisualizationMarkers());

    ROS_INFO_STREAM("iteration: " << iter << " with coverage: " << best_cov_);

    // increment PSO-iterator
    iter++;
  }

}

//GreedyPSO algorithm looks for solution of only one sensor at a time. Therefore, the PSO algorithm is repeatedly run to give result for all sensors.
//One complete run of the PSO algorithm will be referred to as a "PSO round".
void sensor_placement_node::GreedyPSOptimize()
{
  //clock_t t_start;
  //clock_t t_end;
  //double  t_diff;

  //local variable
  double actual_sensor_coverage = 0;
  //PSO-iterator
  int iter = 0;
  std::vector<geometry_msgs::Pose> global_pose;
  //reset total coverage by GreedyPSO
  total_GreedyPSO_covered_targets_num_ = 0;

  //NOTE: One iteration of following loop = One "PSO round"
  //whole swarm optimization step (=PSO round) is iterated as many times as the number of sensors
  for (unsigned int sensor_iter = 0; sensor_iter<sensor_num_; sensor_iter++)
  {
    //reset variables
    iter = 0;
    best_cov_ = 0;
    best_priority_sum_ = 0;

    //place sensors randomly on perimeter for each particle with random velocities
    if(AoI_received_)
    {
      for(size_t i = 0; i < particle_swarm_.size(); i++)
      {
        // initialize sensor poses randomly on perimeter
        particle_swarm_.at(i).placeSensorsRandomlyOnPerimeter();
        // initialize sensor velocities randomly
        particle_swarm_.at(i).initializeRandomSensorVelocities();
        // get calculated coverage
        actual_sensor_coverage = particle_swarm_.at(i).getActualCoverage();
        // check if the actual sensor coverage is a new global best
        if(actual_sensor_coverage > best_cov_)
        {
          best_cov_ = actual_sensor_coverage;
          global_best_ = particle_swarm_.at(i);
        }
      }
      //initialize global best solution
      getGlobalBest();

      //publish the global best visualization
      marker_array_pub_.publish(global_best_.getVisualizationMarkers());
    }

    //iteration step
    //continue calculation as long as there are iteration steps left and actual best coverage (per sensor) is
    //lower than mininmal sensor coverage
    //and goal cancellation is not requested by action client
    while(iter < iter_max_per_sensor_ && best_cov_ < min_sensor_cov_ && !preemptRequested())
    {
      global_pose = global_best_.getSolutionPositions();
      // update each particle in vector
      #pragma omp parallel for
        for(size_t i=0; i < particle_swarm_.size(); i++)
        {
          //t_start = clock();
          particle_swarm_.at(i).resetTargetsWithInfoVar();                  //locked targets are not resetted
          //t_end = clock();
          //t_diff = (double)(t_end - t_start) / (double)CLOCKS_PER_SEC;
          //ROS_INFO( "reset: %10.10f \n", t_diff);

          // now we're ready to update the particle
          //t_start = clock();
          particle_swarm_.at(i).updateParticle(global_pose, PSO_param_1_, PSO_param_2_, PSO_param_3_);
          //t_end = clock();
          //t_diff = (double)(t_end - t_start) / (double)CLOCKS_PER_SEC;
          //ROS_INFO( "updateParticle: %10.10f \n", t_diff);
        }
      //after the update step we're looking for a new global best solution
      getGlobalBest();

      //publish the actual global best visualization
      marker_array_pub_.publish(global_best_.getVisualizationMarkers());

      ROS_INFO_STREAM("sensor number: " << sensor_iter <<" iteration: " << iter << " with coverage: " << best_cov_);

      //increment PSO-iterator
      iter++;
    }
    //break out of for loop if preemption is requested
    if (as_.isPreemptRequested()) break;
    //save the found solution into solution particle
    sol_particle_.setSolutionSensors(global_best_.getActualSolution().at(0));  //global_best_ particle in GreedyPSO has only one sensor
    //publish solution particle
    marker_array_pub_.publish(sol_particle_.getSolutionlVisualizationMarkers());
    //save the coverage by global best particle, first reset targets
    global_best_.resetTargetsWithInfoVar();
    //now lock targets that the global best is covering and count the priority sum for locked targets
    global_best_.updateTargetsInfoRaytracing(0, true);
    //calculate and print total coverage by GreedyPSO
    total_GreedyPSO_covered_targets_num_ = total_GreedyPSO_covered_targets_num_+ global_best_.getNumOfTargetsCovered();
    best_cov_ = (double) total_GreedyPSO_covered_targets_num_/target_num_;
    ROS_INFO_STREAM("Total coverage by GreedyPSO: " << best_cov_);
    //set updated targets for whole particle swarm i.e. make the state of targets_with_info_var
    //TODO: use a pointer for targetsWithInfo instead of each particle having their own targetsWithInfo object
    for(size_t i = 0; i < particle_swarm_.size(); i++)
    {
      particle_swarm_.at(i).setTargetsWithInfoVar(global_best_.getTargetsWithInfoVar());    // priority_sum_ for each particle gets resetted here also
    }
  }
}

// function to run Greedy Search Algorithm
void sensor_placement_node::runGS()
{
  //initialization
  best_cov_ = 0.0;
  double GS_coverage;
  ros::Time start_time;
  ros::Duration end_time;
  std::vector<double> open_angles(2,0);
  bool placement_success = true;

  //start placing sensors one by one according to greedy algorithm
  for(size_t sensor_index = 0; sensor_index < sensor_num_; sensor_index++)
  {
    //note start time for greedy search
    start_time = ros::Time::now();
    //do Greedy Search and place sensor on the max coverage pose
    placement_success = GS_solution.newGreedyPlacement(sensor_index);
    //if placement was preempted, break out of the loop
    if (placement_success == false)
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      action_success_ = false;
      break;
    }
    //note end time for greedy_search
    end_time= ros::Time::now() - start_time;
    //publish the solution
    marker_array_pub_.publish(GS_solution.getVisualizationMarkers());
    //calculate the current coverage
    GS_coverage = GS_solution.calGScoverage();

    ROS_INFO_STREAM("Sensors placed: " << sensor_index+1 << " coverage: " << GS_coverage);
    ROS_INFO_STREAM("Time taken: " << end_time << "[s]");
  }
  best_cov_ = GS_coverage;
}

//new function to find global best particle
void sensor_placement_node::getGlobalBest()
{
  // a new global best solution is accepted if
  // (1) more points of interest are being covered, or if same number of points of interest are being covered, but:
  // (2) the coverage is higher than the old best coverage or
  // (3) the coverage is equal to the old best coverage but there are less targets covered by multiple sensors
  for(size_t i=0; i < particle_swarm_.size(); i++)
  {
    if (particle_swarm_.at(i).getPrioritySum() > best_priority_sum_)
    {
      //update the best_priority_sum_
      best_priority_sum_ = particle_swarm_.at(i).getPrioritySum();
      ROS_INFO_STREAM("Current priority sum: " << best_priority_sum_);

      //update the best solution
      best_cov_ = particle_swarm_.at(i).getActualCoverage();
      global_best_ = particle_swarm_.at(i);
      global_best_multiple_coverage_ = particle_swarm_.at(i).getMultipleCoverageIndex();
      best_particle_index_ = i;
    }

    if (particle_swarm_.at(i).getPrioritySum() == best_priority_sum_)
    {
      if(particle_swarm_.at(i).getActualCoverage() > best_cov_)
      {
        best_cov_ = particle_swarm_.at(i).getActualCoverage();
        global_best_ = particle_swarm_.at(i);
        global_best_multiple_coverage_ = particle_swarm_.at(i).getMultipleCoverageIndex();
        best_particle_index_ = i;
      }
      else
      {
        if( (particle_swarm_.at(i).getActualCoverage() == best_cov_) && (particle_swarm_.at(i).getMultipleCoverageIndex() < global_best_multiple_coverage_ ))
        {
          best_cov_ = particle_swarm_.at(i).getActualCoverage();
          global_best_ = particle_swarm_.at(i);
          global_best_multiple_coverage_ = particle_swarm_.at(i).getMultipleCoverageIndex();
          best_particle_index_ = i;
        }
      }
    }
  }
}

// function to start map service and create look up tables
void sensor_placement_node::initializeCallback()
{
  // call static_map-service from map_server to get the actual map
  sc_get_map_.waitForExistence();

  nav_msgs::GetMap srv_map;

  if(sc_get_map_.call(srv_map))
  {
    ROS_INFO("Map service called successfully");
    map_received_ = true;

    if(AoI_received_)
    {
      // get bounding box of area of interest
      geometry_msgs::Polygon bound_box = getBoundingBox2D(area_of_interest_.polygon, srv_map.response.map);
      // cropMap to boundingBox
      cropMap(bound_box, srv_map.response.map, map_);
    }
    else
    {
      map_ = srv_map.response.map;

      // if no AoI was specified, we consider the whole map to be the AoI
      area_of_interest_.polygon = getBoundingBox2D(geometry_msgs::Polygon(), map_);
    }

    // publish map
    map_.header.stamp = ros::Time::now();
    map_pub_.publish(map_);
    map_meta_pub_.publish(map_.info);
  }
  else
  {
    ROS_INFO("Failed to call map service");
  }

  if(map_received_)
  {
    ROS_INFO("Received a map");

    // now create the lookup table based on the range of the sensor and the resolution of the map
    int radius_in_cells = floor(sensor_range_ / map_.info.resolution);
    lookup_table_ = createLookupTableCircle(radius_in_cells);
  }

}

// callback function for the start PSO service
bool sensor_placement_node::startPSOCallback()
{
  best_cov_ = 0.0;
  //start map service and create look up tables
  initializeCallback();

  ROS_INFO("getting targets from specified map and area of interest!");

  targets_saved_ = getTargets();

  if(targets_saved_)
  {
    ROS_INFO_STREAM("Saved " << target_num_ << " targets in std-vector");

    ROS_INFO_STREAM("Saved " << targets_with_info_fix_.size() << " targets with info in std-vectors");
  }


  ROS_INFO("Initializing particle swarm");
  initializePSO();

  // publish global best visualization
  marker_array_pub_.publish(global_best_.getVisualizationMarkers());

  ROS_INFO("Particle swarm Optimization step");
  PSOptimize();

  // get the PSO result as nav_msgs::Path in UTM coordinates and publish it
  PSO_result_ = particle_swarm_.at(best_particle_index_).particle::getSolutionPositionsAsPath();

  nav_path_pub_.publish(PSO_result_);

  ROS_INFO_STREAM("Print the best solution as Path: " << PSO_result_);

  // publish global best visualization
  marker_array_pub_.publish(global_best_.getVisualizationMarkers());

  ROS_INFO("Clean up everything");
  particle_swarm_.clear();

  targets_with_info_fix_.clear();
  targets_with_info_var_.clear();

  target_num_ = 0;
  best_particle_index_ = 0;
  best_priority_sum_ = 0;

  ROS_INFO("PSO terminated successfully");

  return true;

}

// callback function for the start GreedyPSO service
bool sensor_placement_node::startGreedyPSOCallback()
{
  best_cov_ = 0.0;
  //start map service and create look up tables
  initializeCallback();

  ROS_INFO("getting targets from specified map and area of interest!");

  targets_saved_ = getTargets();

  if(targets_saved_)
  {
    ROS_INFO_STREAM("Saved " << target_num_ << " targets in std-vector");

    ROS_INFO_STREAM("Saved " << targets_with_info_fix_.size() << " targets with info in std-vectors");
  }


  ROS_INFO("Initializing greedy particle swarm");
  initializeGreedyPSO();

  //delete visualization markers from previous run
  marker_array_pub_.publish(sol_particle_.deleteVisualizationMarkers());

  //publish global best visualization
  marker_array_pub_.publish(global_best_.getVisualizationMarkers());

  ROS_INFO("Greedy Particle swarm Optimization step");
  GreedyPSOptimize();

  //optimization completed, now update original sensors vector to get solution as path
  sol_particle_.updateOrigSensorsVec();

  // get the PSO result as nav_msgs::Path in UTM coordinates and publish it
  PSO_result_ = sol_particle_.particle::getSolutionPositionsAsPath();

  nav_path_pub_.publish(PSO_result_);

  ROS_INFO_STREAM("Print the best solution as Path: " << PSO_result_);

  ROS_INFO("Clean up everything");
  particle_swarm_.clear();

  targets_with_info_fix_.clear();
  targets_with_info_var_.clear();

  target_num_ = 0;

  best_particle_index_ = 0;
  best_priority_sum_ = 0;

  ROS_INFO("GreedyPSO terminated successfully");

  return true;

}

// callback function for the start GS service
bool sensor_placement_node::startGSCallback()
{

  //start map service and create look up tables
  initializeCallback();

  ROS_INFO("getting targets from specified map and area of interest!");

  targets_saved_ = getGSTargets();

  if(targets_saved_)
  {
  ROS_INFO_STREAM("Saved " << GS_pool_.size() << " targets in GS pool");
  ROS_INFO_STREAM("Saved " << target_num_ << " all targets");
  ROS_INFO_STREAM("Saved " << point_info_vec_.size() << " all points on map");
  }
  else
  {
    ROS_ERROR("No targets received! Error in getGSTargets function");
    return false;
  }

  ROS_INFO("Initializing Greedy Search algorithm");
  initializeGS();

  // now start the actual Greedy Search  Algorithm
  ROS_INFO("Running Greedy Search Algorithm");
  runGS();

  ROS_INFO("Clean up everything");

  GS_pool_.clear();
  point_info_vec_.clear();

  target_num_ = 0;

  ROS_INFO("Greedy Search Algorithm terminated successfully");

  return true;

}

// callback function for the start GS service with offset parameter
bool sensor_placement_node::startGSWithOffsetCallback()
{
  //start map service and create look up tables
  initializeCallback();

  ROS_INFO("getting targets from specified map and area of interest!");

  targets_saved_ = getGSTargets();

  if(targets_saved_)
  {
  ROS_INFO_STREAM("Saved " << GS_pool_.size() << " targets in GS pool");
  ROS_INFO_STREAM("Saved " << target_num_ << " all targets");
  ROS_INFO_STREAM("Saved " << point_info_vec_.size() << " all points on map");
  }
  else
  {
    ROS_ERROR("No targets received! Error in getGSTargets function");
    return false;
  }

  ROS_INFO("Initializing Greedy Search algorithm");
  initializeGS();

  // now start the actual Greedy Search  Algorithm
  ROS_INFO("Running Greedy Search Algorithm");
  runGS();

  ROS_INFO("Clean up everything");

  //clean up
  polygon_offset_val_received_ = false;
  GS_pool_.clear();
  point_info_vec_.clear();

  target_num_ = 0;

  ROS_INFO("Greedy Search Algorithm terminated successfully");

  return true;

}

// callback function for clearing all forbidden areas
bool sensor_placement_node::clearFACallback()
{
  forbidden_area_vec_.clear();
  visualization_msgs::MarkerArray empty_marker;
  fa_marker_array_pub_.publish(empty_marker);
  fa_received_=false;
  return true;
}

// callback function for the test service
bool sensor_placement_node::testServiceCallback()
{
  // call static_map-service from map_server to get the actual map
  sc_get_map_.waitForExistence();

  nav_msgs::GetMap srv_map;

  if(sc_get_map_.call(srv_map))
  {
    ROS_INFO("Map service called successfully");

    if(AoI_received_)
    {
      // get bounding box of area of interest
      geometry_msgs::Polygon bound_box = getBoundingBox2D(area_of_interest_.polygon, srv_map.response.map);
      // cropMap to boundingBox
      cropMap(bound_box, srv_map.response.map, map_);
      // publish cropped map
      map_.header.stamp = ros::Time::now();
      map_pub_.publish(map_);
      map_meta_pub_.publish(map_.info);
      map_received_ = true;
    }
    else
    {
      // if no AoI was specified, we consider the whole map to be the AoI
      area_of_interest_.polygon = getBoundingBox2D(geometry_msgs::Polygon(), map_);
      map_ = srv_map.response.map;
      map_pub_.publish(map_);
      map_meta_pub_.publish(map_.info);
      map_received_ = true;
    }
  }
  else
  {
    ROS_INFO("Failed to call map service");
  }

  if(map_received_)
  {
    ROS_INFO("Received a map");

    // now create the lookup table based on the range of the sensor and the resolution of the map
    int radius_in_cells = floor(5 / map_.info.resolution);
    lookup_table_ = createLookupTableCircle(radius_in_cells);
  }

  ROS_INFO("getting targets from specified map and area of interest!");

  targets_saved_ = getTargets();
  if(targets_saved_)
  {
    ROS_INFO_STREAM("Saved " << target_num_ << " targets in std-vector");

    ROS_INFO_STREAM("Saved " << targets_with_info_fix_.size() << " targets with info in std-vectors");

  }

  // initialize pointer to dummy sensor_model
  FOV_2D_model dummy_2D_model;
  dummy_2D_model.setMaxVelocity(max_lin_vel_, max_lin_vel_, max_lin_vel_, max_ang_vel_, max_ang_vel_, max_ang_vel_);

  particle_num_ = 1;
  sensor_num_ = 1;
  open_angles_.at(0) = 1.5 * PI;

  // initialize dummy particle
  particle dummy_particle = particle(sensor_num_, target_num_, dummy_2D_model);
  // initialize particle swarm with given number of particles containing given number of sensors

  dummy_particle.setMap(map_);
  dummy_particle.setAreaOfInterest(area_of_interest_);
  dummy_particle.setForbiddenAreaVec(forbidden_area_vec_);
  dummy_particle.setOpenAngles(open_angles_);
  dummy_particle.setRange(5);

  dummy_particle.setTargetsWithInfoFix(targets_with_info_fix_, target_num_);
  dummy_particle.setTargetsWithInfoVar(targets_with_info_var_);

  ROS_INFO_STREAM("creating lookup tables for dummy particle..");
  dummy_particle.setLookupTable(& lookup_table_);
  ROS_INFO_STREAM("lookup tables created.");

  particle_swarm_.assign(particle_num_,dummy_particle);

  // initialze the global best solution
  global_best_ = dummy_particle;

  double actual_coverage = 0;

  // initialize sensors randomly on perimeter for each particle with random velocities
  if(AoI_received_)
  {
    for(size_t i = 0; i < particle_swarm_.size(); i++)
    {
      geometry_msgs::Pose test_pos = geometry_msgs::Pose();
      test_pos.position.x = area_of_interest_.polygon.points.at(0).x+5;
      test_pos.position.y = area_of_interest_.polygon.points.at(0).y+5;
      test_pos.orientation = tf::createQuaternionMsgFromYaw(PI/4);

      particle_swarm_.at(i).placeSensorsAtPos(test_pos);

      global_best_ = particle_swarm_.at(i);

      actual_coverage = global_best_.getActualCoverage();

      ROS_INFO_STREAM("coverage: " << actual_coverage);

      ROS_INFO_STREAM("orientation: " << test_pos.orientation);
      ROS_INFO_STREAM("orientation-map: " << tf::getYaw(map_.info.origin.orientation));
    }
  }
  // publish global best visualization
  marker_array_pub_.publish(global_best_.getVisualizationMarkers());

  return true;
}

// callback function saving the AoI received
void sensor_placement_node::AoICB(const geometry_msgs::PolygonStamped::ConstPtr &AoI)
{
  area_of_interest_ = *AoI;
  AoI_received_ = true;
}

// callback function saving the PoI received
void sensor_placement_node::PoICB(const visualization_msgs::MarkerArray::ConstPtr &PoI)
{
  // save points of interest
  PoI_vec_ = (*PoI).markers.at(0).points;
}

// callback function saving the forbidden area received
void sensor_placement_node::forbiddenAreaCB(const geometry_msgs::PolygonStamped::ConstPtr &forbidden_area)
{
  forbidden_area_vec_.push_back(*forbidden_area);
  fa_received_ = true;
  //NOTE: get visualization function called again when a polygon is received
  fa_marker_array_pub_.publish(getPolygonVecVisualizationMarker(forbidden_area_vec_, "forbidden_area"));
}

//function to calculate a rough approximate of coverage that a sensor can do with a given open angles (in rad) and range (in meters)
unsigned int sensor_placement_node::calculateMaxSensorCoverage(unsigned int range, std::vector<double> open_angles)
{
  //calculate FOV from open angles
  unsigned int FOV = round((open_angles.at(0)-open_angles.at(1))*(180/PI));
  if (FOV<0||FOV>360)
  {
    ROS_ERROR("invalid FOV calculation in greedySearch::calculateMaxSensorCoverage, using default 90 [degrees]");
    FOV = 90;
  }
  //calculate total number of targets in a square that encloses a circle of radius(r) where, r = range of sensor
  unsigned int N = (2 * range/map_.info.resolution) * (2 * range/map_.info.resolution);
  //now calculate a rough upper bound on total targets that the sensor can cover using FOV
  unsigned int max_sensor_coverage = N / ceil(360/FOV);
  if (max_sensor_coverage < 100)
    ROS_INFO_STREAM("WARNING! max_sensor_coverage = " << max_sensor_coverage);
  return max_sensor_coverage;
}

// returns the visualization markers of a vector of polygons
visualization_msgs::MarkerArray sensor_placement_node::getPolygonVecVisualizationMarker(std::vector<geometry_msgs::PolygonStamped> polygons, std::string polygon_name)
{
  visualization_msgs::MarkerArray polygon_marker_array;
  visualization_msgs::Marker line_strip;
  geometry_msgs::Point p;

  for (size_t j=0; j<polygons.size(); j++)
  {
    // setup standard stuff
    line_strip.header.frame_id = std::string("/map");
    line_strip.header.stamp = ros::Time();
    line_strip.ns = polygon_name + boost::lexical_cast<std::string>(j);;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.3;
    line_strip.scale.y = 0.0;
    line_strip.scale.z = 0.0;
    line_strip.color.a = 1.0;
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.1;
    line_strip.color.b = 0.0;

    for (size_t k=0; k<polygons.at(j).polygon.points.size(); k++)
    {
      p.x = polygons.at(j).polygon.points.at(k).x;
      p.y = polygons.at(j).polygon.points.at(k).y;
      line_strip.points.push_back(p);
    }
    //enter first point again to get a closed shape
    line_strip.points.push_back(line_strip.points.at(0));
    //save the marker
    polygon_marker_array.markers.push_back(line_strip);
    //clear line strip point for next polygon
    line_strip.points.clear();
  }
  return polygon_marker_array;
}


//######################
//#### main program ####
int main(int argc, char **argv)
{
  // initialize ros and specify node name
  ros::init(argc, argv, std::string("sensor_placement_node"));

  // create Node Class
  sensor_placement_node my_placement_node;

  // initialize random seed for all underlying classes
  srand(time(NULL));

  // initialize loop rate
  ros::Rate loop_rate(10);

  while(my_placement_node.nh_.ok())
  {
     //can add this later to the funtion below
    ros::spinOnce();

    loop_rate.sleep();
  }

}
