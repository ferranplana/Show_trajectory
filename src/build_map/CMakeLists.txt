cmake_minimum_required(VERSION 2.8.3)
project(build_map)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  geometry_msgs
  tf
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES build_map
#  CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(  ${catkin_INCLUDE_DIRS} )

add_executable(shapes src/shapes.cpp)
add_executable(pose src/pose.cpp)
add_executable(aleatory_pose src/aleatory_pose.cpp)


target_link_libraries(shapes ${catkin_LIBRARIES})
target_link_libraries(pose ${catkin_LIBRARIES})
target_link_libraries(aleatory_pose ${catkin_LIBRARIES})

