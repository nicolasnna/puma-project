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
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include <vector>

// Extra
#include <puma_arduino_msgs/StatusArduino.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
//#include "pid.h"

using namespace std;

namespace puma_local_planner{

  struct position{
    double x, y, z, az;
  };

  struct pid {
    double p, i, d;
    double prev_error, integral_error;
  };

  class PumaLocalPlanner : public nav_core::BaseLocalPlanner{
    public: 
      PumaLocalPlanner();
      PumaLocalPlanner(std::string name, tf2_ros::Buffer* tf,
                        costmap_2d::Costmap2DROS* costmap_ros);
      
      ~PumaLocalPlanner();
      
      void initialize(std::string name, tf2_ros::Buffer* tf,
                      costmap_2d::Costmap2DROS* costmap_ros);
    
      // Manage plan
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
      void setNextPart();
      bool isGoalReached();
      void setErrorNow();
      void setCountPlan();
      
      // Control velocity
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
      void setVelocity();
      void setRotation();
      double computePid(pid, double, double);

      // Callbacks
      void statusArduinoCallback(const puma_arduino_msgs::StatusArduino::ConstPtr &);
      void odometryCallback(const nav_msgs::Odometry::ConstPtr &);

    private:
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_; 
      tf2_ros::Buffer* tf_;
      bool initialized_;

      // Variable direction
      int current_analog_direction;
      float current_angle_direction;

      // Odometry
      position current_position;
      float current_vel_x;
      float current_angular_z;


      // Subs and pub
      ros::Subscriber status_arduino_sub, odometry_sub;
      ros::Publisher ackerman_pub;
      geometry_msgs::Twist cmd;
      ackermann_msgs::AckermannDriveStamped acker_cmd;

      // Plan data
      int length; // Lenght of plan global
      int count; // Number frame of plan global
      double distance;
      std::vector<geometry_msgs::PoseStamped> plan; // it will contain global plan
      bool goal_reached;
      // Pos vel next
      position next_position;
      // Error position
      position error_position;

      // PID Controller
      pid pid_linear, pid_angular;
      double dt;
  };
};


#endif // PUMA_LOCAL_PLANNER_PUMA_LOCAL_PLANNER_H_
