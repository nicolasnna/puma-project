#include <ros/console.h>

#include "buttons_waypoints.h"
#include "std_msgs/Empty.h"

namespace buttons_waypoints {

  ResetPath::ResetPath() {
    ros::NodeHandle nh_;
    publisher_ = nh_.advertise<std_msgs::Empty>("/puma/state_machine/clear_plan", 1);
  }

  ResetPath::~ResetPath() {}

  void ResetPath::onInitialize() {}

  void ResetPath::activate() { 
    Publish();
    // Cambiar a la herramienta "Interact"
    rviz::ToolManager* tool_manager = context_->getToolManager();
    rviz::Tool* interact_tool = tool_manager->getTool(0);
    tool_manager->setCurrentTool(interact_tool);
  }

  void ResetPath::deactivate() {}

  void ResetPath::Publish() {
    std_msgs::Empty msg_ready;
    publisher_.publish(msg_ready);
  }

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(buttons_waypoints::ResetPath, rviz::Tool)