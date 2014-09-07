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
 * Date of creation: August 2013
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

#include <greedySearch.h>

// standard constructor
greedySearch::greedySearch()
{
  // initialize number of sensors
  sensor_num_ = 0;

  // initialize number of targets
  target_num_ = 0;

  // intialize coverage
  coverage_ = 0;

  // intialize covered targets number
  covered_targets_num_ = 0;
}

// constructor with arguments
greedySearch::greedySearch(int num_of_sensors, int num_of_targets, FOV_2D_model sensor_model)
{
  // initialize number of sensors
  sensor_num_ = num_of_sensors;

  // initialze number of targets
  target_num_ = num_of_targets;

  // intialize coverage
  coverage_ = 0;

  // intialize covered targets number
  covered_targets_num_ = 0;

  // initialize sensor vector with as many entries as specified by sensors_num_
  sensors_.assign(sensor_num_, sensor_model);

}

greedySearch::~greedySearch(){}

// function to set action server
void greedySearch::setActionServer(actionlib::SimpleActionServer<seneka_sensor_placement::sensorPlacementAction> * action_server)
{
  // point to the given action server
  as_ = action_server;
}

// function for finding maximum coverage position (using Greedy Search Algorithm) and placing sensor at that position
bool greedySearch::newGreedyPlacement(size_t sensor_index)
{
  unsigned int num_of_slices;
  int coverage;
  geometry_msgs::Pose new_pose;
  std::vector<double> orig_ang_r(2,0);
  std::vector<double> gs_ang_r(2,0);
  std::vector<double> new_ang_r(2,0);
  double slice_res_deg;
  double error_pos;
  double new_sum, max_sum;


  // modify open angles of the sensor for an optimized search around different orientations.
  // This modified open angle is so called "slice" of the original open angles.

  // get slice_open_angles_ parameter
  gs_ang_r = getSliceOpenAngles();
  // convert from radians to degrees
  slice_res_deg = (gs_ang_r[0]-gs_ang_r[1])*(180/PI);
  // make sure results are correct
  if (slice_res_deg > 0 || slice_res_deg < 360)
  {
    // check if the slices evenly fit into 360degrees. if not, adjust slice open angles
    if (fmod(360,slice_res_deg)!=0)
    {
      // **given angle does not fit exactly into 360deg, computing closest angle that does fit**
      error_pos = ceil(360/slice_res_deg)*slice_res_deg - 360;  //value by which (total_slices*slice_resolution) exceeds 360
      if (error_pos<(slice_res_deg/2))
      {
        // new angle must be less than old one to get closest solution
        slice_res_deg = slice_res_deg - error_pos/ceil(360/slice_res_deg);
        ROS_INFO_STREAM("Modified slice open angles[in degrees]: " << slice_res_deg);
      }
      else
      {
        // new angle must be greater than old one to get closest solution
        slice_res_deg = slice_res_deg + ((slice_res_deg-error_pos)/floor(360/slice_res_deg));
        ROS_INFO_STREAM("Modified slice open angles[in degrees]: " << slice_res_deg);
      }
    }
  }
  else
  {
    ROS_ERROR("Invalid evaluation of slice open angles ");
  }

  //save calculated new slice open angles
  gs_ang_r[0] = slice_res_deg*(PI/180);
  //modification complete

  //setup everything before starting the search for optimal sensor pose
  //get original open angles
  orig_ang_r = sensors_.at(sensor_index).getOpenAngles();
  //calculate number of slices that fit into original open angles of the sensor
  num_of_slices = floor((orig_ang_r[0]-orig_ang_r[1])/gs_ang_r[0]);
  //change the sensor open angles for search
  setOpenAngles(gs_ang_r);
  //reset previous max coverage information before searching for new position
  resetMaxSensorCovInfo();
  //reset max_sum
  max_sum=0;

  //now start searching for a optimal pose to place the sensor by considering only a limited set of points i.e. GS_pool_
  //place the current sensor on all points in GS pool one by one and calculate coverge
  for (size_t point_id=0; point_id<GS_pool_.size(); point_id++)
  {
    //clear coverage data from previous loop iteration
    coverage_vec_.clear();

    //calculate world position of current point id
    new_pose.position.x = mapToWorldX(GS_pool_[point_id].x, *pMap_);
    new_pose.position.y = mapToWorldY(GS_pool_[point_id].y, *pMap_);
    new_pose.position.z = 0;
    //check all orientations which are a multiple of new slice open angles
    for (double alpha=0; alpha<2*PI; alpha=alpha+gs_ang_r[0])
    {
      //look around in all directions with resolution of the slice
      new_pose.orientation = tf::createQuaternionMsgFromYaw(alpha);
      sensors_.at(sensor_index).setSensorPose(new_pose);
      coverage = getCoverageRaytracing(sensor_index);         //get coverage at new_pose
      coverage_vec_.push_back(coverage);                      //save in coverage in coverage_vec_

      //check if preemption is requested
      if (as_->isPreemptRequested())
      {
        // set the action state to preempted, if it is śtill active
        if (as_->isActive())
        {
          as_->setPreempted();
        }
        //prepare to return: reset the sensor open angles
        new_ang_r[0] = num_of_slices*gs_ang_r[0];
        setOpenAngles(new_ang_r);
        return false;
      }
    }

    //now find N consecutive slices that give maximum coverage. (where N is num_of_slices)
    for (size_t cov_vec_ind=0; cov_vec_ind<coverage_vec_.size(); cov_vec_ind++)
    {
      //calculate all possible consecutive sums
      new_sum=0;
      for (size_t k = 0; k<num_of_slices; k++)
      {
        new_sum = new_sum + coverage_vec_.at( (cov_vec_ind+k) % (coverage_vec_.size()) );
      }

      //if new sum is larger than max sum then this represents the optimal pose. So, save this pose.
      if (new_sum>max_sum)
      {
        max_sum = new_sum;
        //calculate orientation from coverage vector index
        new_pose.orientation = tf::createQuaternionMsgFromYaw((cov_vec_ind*gs_ang_r[0] + (num_of_slices*gs_ang_r[0])/2)-gs_ang_r[0]/2);
        setMaxSensorCovPOSE(new_pose);
      }
    }
  }
  //search complete, so reset the sensor open angles to a value which is, may be approximately if not exactly, equal to the open angles from parameter server.
  new_ang_r[0] = num_of_slices*gs_ang_r[0];
  setOpenAngles(new_ang_r);
  //place the sensor at max coverage point
  sensors_.at(sensor_index).setSensorPose(getMaxSensorCovPOSE());
  //now update the 'covered' info of the points
  updateCoveredInfoRaytracing(sensor_index);
  return true;
}

//function to get the coverage done by the sensor
int greedySearch::getCoverageRaytracing(size_t sensor_index)
{
  //clear vector of ray end points
  sensors_.at(sensor_index).clearRayEndPoints();

  unsigned int max_number_of_rays = sensors_.at(sensor_index).getLookupTable()->size();
  geometry_msgs::Pose sensor_pose = sensors_.at(sensor_index).getSensorPose();

  std::vector<double> open_ang = sensors_.at(sensor_index).getOpenAngles();
  double orientation = tf::getYaw(sensor_pose.orientation);

  //get angles of sensor and keep them between 0 and 2*PI
  double angle1 = orientation - (open_ang.front() / 2.0);
  if(angle1 >= 2.0*PI)
    angle1 -= 2.0*PI;
  else if(angle1 < 0)
    angle1 += 2.0*PI;

  double angle2 = orientation + (open_ang.front() / 2.0);
  if(angle2 >= 2.0*PI)
    angle2 -= 2.0*PI;
  else if(angle2 < 0)
    angle2 += 2.0*PI;

  unsigned int ray_start = sensors_.at(sensor_index).rayOfAngle(angle1);
  unsigned int ray_end = sensors_.at(sensor_index).rayOfAngle(angle2);

  unsigned int number_of_rays_to_check;

  //are the rays in between the beginning and end of the lookup table?
  if(ray_end >= ray_start)
    number_of_rays_to_check = ray_end - ray_start + 1;
  else
    number_of_rays_to_check = max_number_of_rays - ray_start + ray_end + 1;

  unsigned int rays_checked = 0;
  unsigned int ray = ray_start;

  // initialize coverage by old and new orientaion of the sensor on the current position
  int coverage_by_new_orientation = 0;

  // for a PoI, priority must be added only once, for this following flag is used
  bool just_once = true;

  // list of points of interest
  std::vector<unsigned int> poi_list;
  // list of priority values for the points of interest
  std::vector<unsigned int> priority_value_list;

  //go through all rays
  while(rays_checked < number_of_rays_to_check)
  {
    geometry_msgs::Point ray_end_point;
    ray_end_point.x = 0;
    ray_end_point.y = 0;

    int x, y;
    unsigned int cell;
    int lookup_table_x = 0, lookup_table_y = 0;

    //go through ray
    for(cell=0; cell < sensors_.at(sensor_index).getLookupTable()->at(ray).size(); cell++)
    {
      lookup_table_x = sensors_.at(sensor_index).getLookupTable()->at(ray).at(cell).x;
      lookup_table_y = sensors_.at(sensor_index).getLookupTable()->at(ray).at(cell).y;

      //absolute x and y map coordinates of the current cell
      x = worldToMapX(sensor_pose.position.x, *pMap_) + lookup_table_x;
      y = worldToMapY(sensor_pose.position.y, *pMap_) + lookup_table_y;

      unsigned int cell_in_vector_coordinates = y * pMap_->info.width + x;


      //cell coordinates are valid (not outside of the area of interest)
      if(((y >= 0) && (x >= 0) && (y < (int) pMap_->info.height) && (x < (int) pMap_->info.width)) && (cell_in_vector_coordinates < pPoint_info_vec_->size()))
      {
        //cell not on the perimeter
        if(pPoint_info_vec_->at(cell_in_vector_coordinates).potential_target != 0)
        {
          //cell a potential target and not occupied
          if((pPoint_info_vec_->at(cell_in_vector_coordinates).potential_target == 1) &&
             (pPoint_info_vec_->at(cell_in_vector_coordinates).occupied == false))
          {
            //cell not already covered
            if(pPoint_info_vec_->at(cell_in_vector_coordinates).covered == false)
            {
              coverage_by_new_orientation++;

              if (pPoint_info_vec_->at(cell_in_vector_coordinates).priority > 0)
              {
                coverage_by_new_orientation+=pPoint_info_vec_->at(cell_in_vector_coordinates).priority;
                //once priority is added, make it 0 and save it for later restoration
                //NOTE: beware that if this function is executed on multiple threads, then it may result into incorrect behaviour
                poi_list.push_back(cell_in_vector_coordinates);
                priority_value_list.push_back(pPoint_info_vec_->at(cell_in_vector_coordinates).priority);
                pPoint_info_vec_->at(cell_in_vector_coordinates).priority = 0;
              }
            }
          }
          //cell not a potential target or occupied -> skip rest of this ray
          else
          {
            break;
          }
        }
        else
        {
          //cell on perimeter and not occupied -> continue with the next cell on the ray (no coverage)
          if(pPoint_info_vec_->at(cell_in_vector_coordinates).occupied == false)
          {
            //continue with next cell without (no coverage)
            continue;
          }
          else
          //cell on perimeter and occupied -> skip rest of this ray
          {
            break;
          }
        }
      }
      else
      //cell coordinates not valid (outside the area of interest) -> skip rest of this ray
      {
        break;
      }
    }

    //skipped some part of the ray -> get coordinates of the last non-occupied cell
    if((cell != sensors_.at(sensor_index).getLookupTable()->at(ray).size()-1) && (cell != 0))
    {
      lookup_table_x = sensors_.at(sensor_index).getLookupTable()->at(ray).at(cell-1).x;
      lookup_table_y = sensors_.at(sensor_index).getLookupTable()->at(ray).at(cell-1).y;
    }

    //absolute x and y map coordinates of the last non-occupied cell
    x = worldToMapX(sensor_pose.position.x, *pMap_) + lookup_table_x;
    y = worldToMapY(sensor_pose.position.y, *pMap_) + lookup_table_y;

    //update endpoint
    if(lookup_table_x <= 0)
      //point is left of sensor
    {
      ray_end_point.x = mapToWorldX(x, *pMap_) - sensor_pose.position.x;
    }
    else
      //cell is right of sensor
    {
      ray_end_point.x = mapToWorldX(x, *pMap_) - sensor_pose.position.x + pMap_->info.resolution; //add one cell for visualization
    }

    if(lookup_table_y <= 0)
      //cell is below sensor
    {
      ray_end_point.y = mapToWorldY(y, *pMap_) - sensor_pose.position.y;
    }
    else
      //cell is over sensor
    {
      ray_end_point.y = mapToWorldY(y, *pMap_) - sensor_pose.position.y + pMap_->info.resolution; //add one cell for visualization
    }

    //add endpoint to the vector of endpoints
    sensors_.at(sensor_index).addRayEndPoint(ray_end_point);

    //increase counter
    rays_checked++;

    //reached end of circle -> set ray to 0
    if(ray == (max_number_of_rays -1))
    {
      ray = 0;
    }
    else
    {
      ray++;
    }
  }
  //all rays checked

  //now restore the deleted priorities
  for (size_t i=0; i<poi_list.size(); i++)
  {
    pPoint_info_vec_->at(poi_list.at(i)).priority = priority_value_list.at(i);
  }


  return coverage_by_new_orientation;
}

//function to update the covered info of the points (i.e. targets)
void greedySearch::updateCoveredInfoRaytracing(size_t sensor_index)
{
  if(pPoint_info_vec_ == NULL)
    return;

  //clear vector of ray end points
  sensors_.at(sensor_index).clearRayEndPoints();

  unsigned int max_number_of_rays = sensors_.at(sensor_index).getLookupTable()->size();
  geometry_msgs::Pose sensor_pose = sensors_.at(sensor_index).getSensorPose();

  std::vector<double> open_ang = sensors_.at(sensor_index).getOpenAngles();
  double orientation = tf::getYaw(sensor_pose.orientation);

  //get angles of sensor and keep them between 0 and 2*PI
  double angle1 = orientation - (open_ang.front() / 2.0);
  if(angle1 >= 2.0*PI)
    angle1 -= 2.0*PI;
  else if(angle1 < 0)
    angle1 += 2.0*PI;

  double angle2 = orientation + (open_ang.front() / 2.0);
  if(angle2 >= 2.0*PI)
    angle2 -= 2.0*PI;
  else if(angle2 < 0)
    angle2 += 2.0*PI;

  unsigned int ray_start = sensors_.at(sensor_index).rayOfAngle(angle1);
  unsigned int ray_end = sensors_.at(sensor_index).rayOfAngle(angle2);

  unsigned int number_of_rays_to_check;

  //are the rays in between the beginning and end of the lookup table?
  if(ray_end >= ray_start)
    number_of_rays_to_check = ray_end - ray_start + 1;
  else
    number_of_rays_to_check = max_number_of_rays - ray_start + ray_end + 1;

  unsigned int rays_checked = 0;
  unsigned int ray = ray_start;

  //go through all rays
  while(rays_checked < number_of_rays_to_check)
  {
    geometry_msgs::Point ray_end_point;
    ray_end_point.x = 0;
    ray_end_point.y = 0;

    int x, y;
    unsigned int cell;
    int lookup_table_x = 0, lookup_table_y = 0;

    //go through ray
    for(cell=0; cell < sensors_.at(sensor_index).getLookupTable()->at(ray).size(); cell++)
    {
      lookup_table_x = sensors_.at(sensor_index).getLookupTable()->at(ray).at(cell).x;
      lookup_table_y = sensors_.at(sensor_index).getLookupTable()->at(ray).at(cell).y;

      //absolute x and y map coordinates of the current cell
      x = worldToMapX(sensor_pose.position.x, *pMap_) + lookup_table_x;
      y = worldToMapY(sensor_pose.position.y, *pMap_) + lookup_table_y;

      unsigned int cell_in_vector_coordinates = y * pMap_->info.width + x;

      //cell coordinates are valid (not outside of the area of interest)
      if(((y >= 0) && (x >= 0) && (y < (int) pMap_->info.height) && (x < (int) pMap_->info.width)) && (cell_in_vector_coordinates < pPoint_info_vec_->size()))
      {
        //cell not on the perimeter
        if(pPoint_info_vec_->at(cell_in_vector_coordinates).potential_target != 0)
        {
          //cell a potential target and not occupied
          if((pPoint_info_vec_->at(cell_in_vector_coordinates).potential_target == 1) &&
             (pPoint_info_vec_->at(cell_in_vector_coordinates).occupied == false))
          {
            //cell not already covered
            if(pPoint_info_vec_->at(cell_in_vector_coordinates).covered == false)
            {
              //mark this cell as covered as the current position of sensor is the final placement position
              pPoint_info_vec_->at(cell_in_vector_coordinates).covered = true;
              covered_targets_num_++;
            }
          }
          //cell not a potential target or occupied -> skip rest of this ray
          else
          {
            break;
          }
        }
        else
        {
          //cell on perimeter and not occupied -> continue with the next cell on the ray (no coverage)
          if(pPoint_info_vec_->at(cell_in_vector_coordinates).occupied == false)
          {
            //continue with next cell without (no coverage)
            continue;
          }
          else
          //cell on perimeter and occupied -> skip rest of this ray
          {
            break;
          }
        }
      }
      else
      //cell coordinates not valid (outside the area of interest) -> skip rest of this ray
      {
        break;
      }
    }

    //skipped some part of the ray -> get coordinates of the last non-occupied cell
    if((cell != sensors_.at(sensor_index).getLookupTable()->at(ray).size()-1) && (cell != 0))
    {
      lookup_table_x = sensors_.at(sensor_index).getLookupTable()->at(ray).at(cell-1).x;
      lookup_table_y = sensors_.at(sensor_index).getLookupTable()->at(ray).at(cell-1).y;
    }

    //absolute x and y map coordinates of the last non-occupied cell
    x = worldToMapX(sensor_pose.position.x, *pMap_) + lookup_table_x;
    y = worldToMapY(sensor_pose.position.y, *pMap_) + lookup_table_y;

    //update endpoint
    if(lookup_table_x <= 0)
      //point is left of sensor
    {
      ray_end_point.x = mapToWorldX(x, *pMap_) - sensor_pose.position.x;
    }
    else
      //cell is right of sensor
    {
      ray_end_point.x = mapToWorldX(x, *pMap_) - sensor_pose.position.x + pMap_->info.resolution; //add one cell for visualization
    }

    if(lookup_table_y <= 0)
      //cell is below sensor
    {
      ray_end_point.y = mapToWorldY(y, *pMap_) - sensor_pose.position.y;
    }
    else
      //cell is over sensor
    {
      ray_end_point.y = mapToWorldY(y, *pMap_) - sensor_pose.position.y + pMap_->info.resolution; //add one cell for visualization
    }

    //add endpoint to the vector of endpoints
    sensors_.at(sensor_index).addRayEndPoint(ray_end_point);

    //increase counter
    rays_checked++;

    //reached end of circle -> set ray to 0
    if(ray == (max_number_of_rays -1))
    {
      ray = 0;
    }
    else
    {
      ray++;
    }
  }
  //all rays checked
}

// function to calculate coverage achieved
double greedySearch::calGScoverage()
{
  // calculate coverage percentage
  coverage_ = (double) covered_targets_num_ / target_num_;

  return coverage_;
}

// function to get maximum sensor coverage pose
geometry_msgs::Pose greedySearch::getMaxSensorCovPOSE()
{
  return max_sensor_cov_pose_;
}

// function to get sensor's FOV slice open angles
std::vector<double> greedySearch::getSliceOpenAngles()
{
  return slice_open_angles_;
}

// function to set maximum sensor coverage pose
void greedySearch::setMaxSensorCovPOSE(geometry_msgs::Pose sensor_pose)
{
  max_sensor_cov_pose_ = sensor_pose;
}

// function to set the information for all targets (point_info_vec_)
void greedySearch::setPointInfoVec(std::vector<point_info> & point_info_vec, int target_num)
{
  pPoint_info_vec_ = &point_info_vec;
  target_num_ = target_num;
  if (pPoint_info_vec_ == NULL)
    ROS_ERROR("point_info_vec not set correctly");
}

// function to set the information for GS pool
void greedySearch::setGSpool(const std::vector<GS_point> &GS_pool)
{
  GS_pool_ = GS_pool;
  covered_targets_num_ = 0;
}

// function that set the map
void greedySearch::setMap(const nav_msgs::OccupancyGrid & new_map)
{
  pMap_ = &new_map;
  if (pMap_ == NULL)
    ROS_ERROR("Map was not set correctly.");
}

// function that sets the area of interest
void greedySearch::setAreaOfInterest(const geometry_msgs::PolygonStamped & new_poly)
{
  pArea_of_interest_ = & new_poly;
  if (pArea_of_interest_ == NULL)
    ROS_ERROR("AoI was not set correctly.");
}

// function that sets forbidden areas vector
void greedySearch::setForbiddenAreaVec(const std::vector<geometry_msgs::PolygonStamped> & new_forbidden_area_vec_)
{
  pForbidden_poly_ = & new_forbidden_area_vec_;
  if (pForbidden_poly_ == NULL)
    ROS_ERROR("Forbidden Area vector was not set correctly.");
}

// function that sets the opening angles for each sensor
bool greedySearch::setOpenAngles(std::vector<double> new_angles)
{
  bool result = false;
  if(new_angles.empty() || (new_angles.size() != 2) )
  {
    ROS_WARN("wrong input in greedySearch::setOpenAngles!");
    return result;
  }
  else
  {
    for(size_t i = 0; i < sensors_.size(); i++)
    {
      sensors_.at(i).setOpenAngles(new_angles.at(0), new_angles.at(1));
    }
    result = true;
    return result;
  }
}

// function that sets sensor's FOV slice open anlges
bool greedySearch::setSliceOpenAngles(std::vector<double> new_angles)
{
  bool result = false;
  if(new_angles.empty() || (new_angles.size() != 2) )
  {
    ROS_WARN("wrong input in greedySearch::setSliceOpenAngles!");
    return result;
  }
  else
  {
    slice_open_angles_ = new_angles;
    result = true;
    return result;
  }
}

// function that sets the range for each sensor
void greedySearch::setRange(double new_range)
{
  for(size_t i = 0; i < sensors_.size(); i++)
  {
    sensors_.at(i).setRange(new_range);
  }
}

// function to create and set a lookup table for raytracing for each sensor in the greedySearch solution
void greedySearch::setLookupTable(const std::vector< std::vector<geometry_msgs::Point32> > * pLookup_table)
{
  if (pLookup_table != NULL)
  {
    for(size_t i = 0; i < sensors_.size(); i++)
    {
      sensors_.at(i).setLookupTable(pLookup_table);
    }
  }
  else
    ROS_ERROR("LookupTable not set correctly");
}

// function to reset maximum coverage information for new sensor placement
void greedySearch::resetMaxSensorCovInfo()
{
  geometry_msgs::Pose reset_pose;
  reset_pose.position.x = 0;
  reset_pose.position.y = 0;
  reset_pose.position.z = 0;
  reset_pose.orientation = tf::createQuaternionMsgFromYaw(0);
  setMaxSensorCovPOSE(reset_pose);
}

// returns all visualization markers of the greedySearch solution
visualization_msgs::MarkerArray greedySearch::getVisualizationMarkers()
{
  visualization_msgs::MarkerArray array, tmp;
  std::vector<FOV_2D_model>::iterator it;
  unsigned int id = 0;
  // loop over all sensors
  for ( it = sensors_.begin(); it != sensors_.end(); ++it )
  {
    tmp = it->getVisualizationMarkers(id);
    // copy over all markers
    for (unsigned int i = 0; i < tmp.markers.size(); i++)
      array.markers.push_back(tmp.markers.at(i));

    id++;
  }
  return array;
}

// returns the visualization markers of points in GS_pool_
visualization_msgs::MarkerArray greedySearch::getGridVisualizationMarker()
{
  visualization_msgs::MarkerArray grids_array;
  visualization_msgs::Marker t_points;
  geometry_msgs::Point p;
  size_t GS_poolsize = GS_pool_.size();
  unsigned int visualization_size_set = 7;

  for (unsigned int i=0; i<visualization_size_set; i++)
  {
    // setup standard stuff
    t_points.header.frame_id = "/map";
    t_points.header.stamp = ros::Time();
    t_points.ns = "grid" + boost::lexical_cast<std::string>(i);;
    t_points.action = visualization_msgs::Marker::ADD;
    t_points.pose.orientation.w = 1.0;
    t_points.id = 0;
    t_points.type = visualization_msgs::Marker::POINTS;
    t_points.scale.x = 0.2+0.1*i;
    t_points.scale.y = 0.2+0.1*i;
    t_points.color.a = 1.0;
    t_points.color.r = 0.0;
    t_points.color.g = 0.0;
    t_points.color.b = 1.0;

    for (size_t point_id=0; point_id<GS_poolsize; point_id++)
    {
      p.x = mapToWorldX(GS_pool_[point_id].x, *pMap_);
      p.y = mapToWorldY(GS_pool_[point_id].y, *pMap_);
      t_points.points.push_back(p);
    }
    grids_array.markers.push_back(t_points);
  }
  return grids_array;
}
