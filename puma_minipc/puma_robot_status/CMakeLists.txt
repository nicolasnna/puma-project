cmake_minimum_required(VERSION 3.0.2)
project(puma_robot_status)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  puma_msgs
  rospy
  std_msgs
  sensor_msgs
  actionlib_msgs
  actionlib
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES
  RobotStatistics.action
  LoggerManager.action
  LightsManager.action
  ChargeManager.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  puma_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES puma_robot_status
  CATKIN_DEPENDS geometry_msgs nav_msgs puma_msgs rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/puma_robot_status.cpp
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(${PROJECT_NAME}_node src/puma_robot_status_node.cpp)

# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

catkin_install_python(PROGRAMS
  scripts/battery_status_node.py
  scripts/robot_statistics_node.py
  scripts/logger_manager_node.py
  scripts/lights_manager_node.py
  scripts/charge_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# catkin_add_gtest(${PROJECT_NAME}-test test/test_puma_robot_status.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# catkin_add_nosetests(test)
