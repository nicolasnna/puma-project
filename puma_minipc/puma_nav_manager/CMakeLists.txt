cmake_minimum_required(VERSION 3.0.2)
project(puma_nav_manager)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  puma_msgs
  nav_msgs
  actionlib_msgs
  actionlib
  message_generation
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
catkin_python_setup()

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  ImportExportPlan.action
  WaypointsManager.action
  LocalizationManager.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  puma_msgs
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy message_runtime
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/nav_manager_node.py
  scripts/manager_localization_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
