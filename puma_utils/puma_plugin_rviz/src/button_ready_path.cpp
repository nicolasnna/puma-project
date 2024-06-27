#include <ros/console.h>

#include "buttons_waypoints.h"
#include "std_msgs/Empty.h"

namespace buttons_waypoints {

  ReadyPath::ReadyPath() {
    ros::NodeHandle nh_;
    publisher_ = nh_.advertise<std_msgs::Empty>("/puma/waypoints/plan_ready", 1);
  }

  ReadyPath::~ReadyPath() {}

  void ReadyPath::onInitialize() {}

  void ReadyPath::activate() { 
    Publish();
  }

  void ReadyPath::deactivate() {}

  void ReadyPath::Publish() {
    std_msgs::Empty msg_ready;
    publisher_.publish(msg_ready);
  }

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(buttons_waypoints::ReadyPath, rviz::Tool)