#include <ros/console.h>

#include "buttons_waypoints.h"
#include "std_msgs/String.h"

namespace buttons_waypoints
{

  UploadPlan::UploadPlan()
  {
    ros::NodeHandle nh_;
    publisher_ = nh_.advertise<std_msgs::String>("/puma/waypoints/plan_upload", 1);
  }

  UploadPlan::~UploadPlan() {}

  void UploadPlan::onInitialize() {}

  void UploadPlan::activate()
  {
    Publish();
    // Cambiar a la herramienta "Interact"
    rviz::ToolManager *tool_manager = context_->getToolManager();
    rviz::Tool *interact_tool = tool_manager->getTool(0);
    tool_manager->setCurrentTool(interact_tool);
  }

  void UploadPlan::deactivate() {}

  void UploadPlan::Publish()
  {
    std_msgs::String msg_upload;
    msg_upload.data = "default_plan";
    publisher_.publish(msg_upload);
  }

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(buttons_waypoints::UploadPlan, rviz::Tool)