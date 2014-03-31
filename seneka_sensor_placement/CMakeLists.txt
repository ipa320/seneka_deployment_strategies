cmake_minimum_required(VERSION 2.8.3)
project(seneka_sensor_placement)

# set build type
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "+++ setting CMAKE_BUILD_TYPE to Release +++")
  set(CMAKE_BUILD_TYPE Release)  
endif(NOT CMAKE_BUILD_TYPE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rosconsole
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf
  tf_conversions
  std_srvs
  message_generation
  actionlib
  actionlib_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenMP REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    

add_service_files(
  DIRECTORY
    srv
  FILES
    polygon_offset.srv
)

add_action_files(
  DIRECTORY
    action
   FILES
    sensorPlacement.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
    common/include
    third_party/clipper_library/include
#  LIBRARIES seneka
  CATKIN_DEPENDS
    roslib
    roscpp
    rosconsole
    std_msgs
    nav_msgs
    message_runtime
    geometry_msgs
    visualization_msgs
    tf
    std_srvs
    actionlib_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
  include/seneka_sensor_placement
  common/include/seneka_sensor_placement
  third_party/clipper_library/include/clipper
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


## Declare a cpp executable
# add_executable(seneka_node src/seneka_node.cpp)
add_executable(sensor_placement_node 	src/sensor_placement_node.cpp
					common/src/particle.cpp
					common/src/seneka_utilities.cpp
					common/src/sensor_model.cpp
					common/src/greedySearch.cpp
					third_party/clipper_library/src/clipper.cpp
					)
add_executable(sensor_placement_test	test/sensor_placement_test_publisher.cpp)
add_executable(sensor_placement_interface src/sensor_placement_interface.cpp)

add_dependencies(sensor_placement_node  ${PROJECT_NAME}_gencpp)
add_dependencies(sensor_placement_interface  ${PROJECT_NAME}_gencpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(sensor_placement_node
   ${catkin_LIBRARIES}
 )
 target_link_libraries(sensor_placement_test
   ${catkin_LIBRARIES}
 )
 
 target_link_libraries(sensor_placement_interface
  ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
 install(TARGETS sensor_placement_node sensor_placement_test
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

