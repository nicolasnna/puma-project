/**************************************
*     Local planner custom for Puma   *
*         Nicolas Norambuena          *
***************************************/
#ifndef PUMA_LOCAL_PLANNER_H_
#define PUMA_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/odometry_helper_ros.h>

using namespace std;

namespace puma_local_planner{

class PumaLocalPlanner : public nav_core::BaseLocalPlanner{
public: 
  PumaLocalPlanner();
  PumaLocalPlanner(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);
  
  ~PumaLocalPlanner();
  
  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros);
 
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
   
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  bool isGoalReached();

private:
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_; 
  tf2_ros::Buffer* tf_;
  bool initialized_;

  // Odometry
  std::shared_ptr<base_local_planner::OdometryHelperRos> odom_helper_;

};
};


#endif // PUMA_LOCAL_PLANNER_PUMA_LOCAL_PLANNER_H_
