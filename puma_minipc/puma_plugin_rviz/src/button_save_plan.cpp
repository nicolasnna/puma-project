#include <ros/console.h>

#include "buttons_waypoints.h"
#include "std_msgs/String.h"

namespace buttons_waypoints
{

  SavePlan::SavePlan()
  {
    ros::NodeHandle nh_;
    publisher_ = nh_.advertise<std_msgs::String>("/puma/waypoints/plan_save", 1);
  }

  SavePlan::~SavePlan() {}

  void SavePlan::onInitialize() {}

  void SavePlan::activate()
  {
    Publish();
    // Cambiar a la herramienta "Interact"
    rviz::ToolManager *tool_manager = context_->getToolManager();
    rviz::Tool *interact_tool = tool_manager->getTool(0);
    tool_manager->setCurrentTool(interact_tool);
  }

  void SavePlan::deactivate() {}

  void SavePlan::Publish()
  {
    std_msgs::String msg_save;
    msg_save.data = "default_plan";
    publisher_.publish(msg_save);
  }

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(buttons_waypoints::SavePlan, rviz::Tool)