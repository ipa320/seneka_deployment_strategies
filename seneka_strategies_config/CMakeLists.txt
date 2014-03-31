cmake_minimum_required(VERSION 2.8.3)
project(seneka_strategies_config)

# set build type
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "+++ setting CMAKE_BUILD_TYPE to Release +++")
  set(CMAKE_BUILD_TYPE Release)  
endif(NOT CMAKE_BUILD_TYPE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES seneka_strategies_config
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
   config/sensor_placement.yaml
   config/seneka_rviz_groovy_config.rviz
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

