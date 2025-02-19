cmake_minimum_required(VERSION 3.0.2)
project(puma_local_planner)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  nav_core
  nav_msgs
  pluginlib
  geometry_msgs
  roscpp
  tf
  visualization_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/PumaLocalPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES puma_local_planner
  CATKIN_DEPENDS geometry_msgs roscpp tf visualization_msgs base_local_planner costmap_2d nav_core nav_msgs pluginlib dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/puma_local_planner.cpp)
add_library(dubins src/dubins.c)

target_link_libraries(puma_local_planner 
  ${catkin_LIBRARIES}
  dubins
)

install(TARGETS puma_local_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES puma_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)