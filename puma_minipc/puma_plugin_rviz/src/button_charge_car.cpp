#include <ros/console.h>

#include "buttons_waypoints.h"
#include "std_msgs/Empty.h"

namespace buttons_waypoints
{

  ChargeCar::ChargeCar()
  {
    ros::NodeHandle nh_;
    publisher_ = nh_.advertise<std_msgs::Empty>("/puma/state_machine/run_charge_mode", 1);
  }

  ChargeCar::~ChargeCar() {}

  void ChargeCar::onInitialize() {}

  void ChargeCar::activate()
  {
    Publish();
    // Cambiar a la herramienta "Interact"
    rviz::ToolManager *tool_manager = context_->getToolManager();
    rviz::Tool *interact_tool = tool_manager->getTool(0);
    tool_manager->setCurrentTool(interact_tool);
  }

  void ChargeCar::deactivate() {}

  void ChargeCar::Publish()
  {
    std_msgs::Empty msg_charge;
    publisher_.publish(msg_charge);
  }

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(buttons_waypoints::ChargeCar, rviz::Tool)