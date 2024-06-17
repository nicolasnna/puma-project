#include "puma_local_planner.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <base_local_planner/goal_functions.h>
#include <memory>


using namespace std;

// Register planner
PLUGINLIB_EXPORT_CLASS(puma_local_planner::PumaLocalPlanner, nav_core::BaseLocalPlanner);

namespace puma_local_planner {


PumaLocalPlanner::PumaLocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

PumaLocalPlanner::PumaLocalPlanner(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false) {
  initialize(name, tf, costmap_ros);
}
 
PumaLocalPlanner::~PumaLocalPlanner() {}

void PumaLocalPlanner::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS* costmap_ros) {
  if(!initialized_) {
    ros::NodeHandle nh = ros::NodeHandle("~" + name);
    tf_ = tf;
    // std::shared_ptr<base_local_planner::OdometryHelperRos> odom_helper_;

    // odom_helper_ = std::make_shared<base_local_planner::OdometryHelperRos>("/odometry/filtered");


    costmap_ros_ = costmap_ros;
    // initialize the copy of the costmap the controller will use
    costmap_ = costmap_ros_->getCostmap();
   
    ROS_INFO("Puma Local planner iniciado!");
    initialized_ = true;
  } else {
    ROS_WARN("Puma Local planner ya se encuentra iniciado!");
  }
}

bool PumaLocalPlanner::setPlan( const std::vector<geometry_msgs::PoseStamped>& orig_global_plan ) {
  if(!initialized_) {
    ROS_ERROR("Este planificador no ha sido iniciado");
    return false;
  }
  return true;
}

bool PumaLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if(!initialized_) {
    ROS_ERROR("Este planificador no ha sido iniciado");
    return false;
  }
  return true;
}

bool PumaLocalPlanner::isGoalReached() {
  if(!initialized_) {
    ROS_ERROR("Este planificador no ha sido iniciado");
    return false;
  }
  return true;
}

} // namespace puma_local_planner