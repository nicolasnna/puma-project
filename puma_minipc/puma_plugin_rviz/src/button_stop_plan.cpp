#include <ros/console.h>

#include "buttons_waypoints.h"
#include "std_msgs/Empty.h"

namespace buttons_waypoints {

  StopPlan::StopPlan() {
    ros::NodeHandle nh_;
    publisher_ = nh_.advertise<std_msgs::Empty>("/puma/waypoints/plan_stop", 1);
  }

  StopPlan::~StopPlan() {}

  void StopPlan::onInitialize() {}

  void StopPlan::activate() { 
    Publish();
    // Cambiar a la herramienta "Interact"
    rviz::ToolManager* tool_manager = context_->getToolManager();
    rviz::Tool* interact_tool = tool_manager->getTool(0);
    tool_manager->setCurrentTool(interact_tool);
  }

  void StopPlan::deactivate() {}

  void StopPlan::Publish() {
    std_msgs::Empty msg_ready;
    publisher_.publish(msg_ready);
  }

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(buttons_waypoints::StopPlan, rviz::Tool)