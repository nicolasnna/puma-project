cmake_minimum_required(VERSION 3.0.2)
project(puma_global_planner)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  puma_msgs
  nav_core
  pluginlib
  dynamic_reconfigure
  roscpp
  tf
)

generate_dynamic_reconfigure_options(
  cfg/PumaGlobalPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES puma_global_planner
  CATKIN_DEPENDS roscpp costmap_2d geometry_msgs nav_core pluginlib tf dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/puma_global_planner.cpp
)

add_library(dubins 
  src/dubins.c)

target_link_libraries(puma_global_planner
  ${catkin_LIBRARIES}
  dubins
)

install(TARGETS puma_global_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIN_DESTINATION}
)
install(FILES puma_global_planner.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
