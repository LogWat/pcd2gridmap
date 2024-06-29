cmake_minimum_required(VERSION 3.0.2)
project(pcd2gridmap)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp std_msgs nav_msgs pcl_ros
)

###########
## Build ##
###########
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcd2gridmap src/pcd2gridmap.cpp)
target_link_libraries(pcd2gridmap ${catkin_LIBRARIES})
