/**
 * Local planner for PUMA robot
 * created by: Nicolas Norambuena
 */

#ifndef PUMA_LOCAL_PLANNER_H
#define PUMA_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <puma_local_planner/PumaLocalPlannerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>


struct Position {
  double x, y, yaw;
  double vel_x, ang_z;
  /* Constructor */
  Position(double x, double y, double yaw) : x(x), y(y), yaw(yaw), vel_x(NAN), ang_z(NAN) {}
  Position() : x(NAN), y(NAN), yaw(NAN), vel_x(NAN), ang_z(NAN)  {}
  Position(double x, double y, double yaw, double vel_x, double ang_z) : x(x), y(y), yaw(yaw), vel_x(vel_x), ang_z(ang_z)  {}
};

namespace puma_local_planner {
  class PumaLocalPlanner : public nav_core::BaseLocalPlanner {
    public: 
      PumaLocalPlanner();
      PumaLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
      ~PumaLocalPlanner();
      /* Sobreescribir funciones existentes */
      void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
      bool isGoalReached();
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

    private: 
      /* Cargar parametros */
      void loadParameters(ros::NodeHandle& nh);
      void reconfigureCB(puma_local_planner::PumaLocalPlannerConfig &config, uint32_t level);

      void odometryCb(const nav_msgs::Odometry::ConstPtr& msg);

      /* Manejo del plan */
      void updatePlan();
      void publishLocalPath(std::vector<Position> path);
      std::vector<Position> simulatePaths();
      double calculateCost(std::vector<Position> path);
      std::vector<Position> simulateReversePaths();
      double calculateReverseCost(std::vector<Position> path, std::vector<Position> reference);

      /* Utilidades */
      double calculateMaxAllowedVelocity();
      double normalizeAngle(const double angle);
      std::vector<Position> generatePath(double vel, double angle);
      void getAdjustXYCostmap(double pos_x, double pos_y, int& cell_x, int& cell_y);

      /* Verificaciones */
      bool isValidPose(const Position& pos);
      bool validateGoalReached(double goal_x, double goal_y, double current_x, double current_y);

      /* Marcadores de rutas */
      visualization_msgs::Marker createPathMarker(const std::vector<Position>& path, int id);
      visualization_msgs::Marker createDeleteAllMarker() const;

      /* Atributos heredados */
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_;
      tf2_ros::Buffer* tf_;
      bool initialized_;

      /* Parámetros */
      std::string topic_odom_;
      int max_index_path_compare_;
      double turning_radius_;
      double max_velocity_, min_velocity_;
      double steering_rads_limit_;
      double max_acceleration_, max_deceleration_, distance_for_deceleration_;
      int steering_samples_, velocity_samples_;
      double time_simulation_, time_step_;
      double time_simulation_reverse_;
      double xy_goal_tolerance_;
      double factor_velocity_;
      double distance_reverse_;

      /* Atributos */
      bool setup_;
      puma_local_planner::PumaLocalPlannerConfig default_config_;
      ros::Subscriber odometry_sub_;
      ros::Publisher path_global_pub_, path_local_pub_, trajectory_pub_;
      bool reversing_mode_;
      bool forward_navigation_failed_;
      bool backward_navigation_failed_;
      Position puma_, goal_;
      Position init_reverse_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
  };
};

#endif // PUMA_LOCAL_PLANNER_H