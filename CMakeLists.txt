cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_frame_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED pcl_ros geometry_msgs pcl_conversions)
find_package(PCL)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES people_tracking
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

## Declare a cpp library
# add_library(people_tracking
#   src/${PROJECT_NAME}/people_tracking.cpp
# )

## Declare a cpp executable
add_executable(point_cloud_frame_test src/point_cloud_frame_test.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(people_tracking_node people_tracking_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(point_cloud_frame_test
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)
