#ifndef PUMA_DWA_LOCAL_PLANNER_H
#define PUMA_DWA_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/tf.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>

struct Position {
  double x, y, yaw;
  double vel_x;
  Position() : x(NAN), y(NAN), yaw(NAN), vel_x(NAN) {}
  Position(double x_, double y_, double yaw_) : x(x_), y(y_), yaw(yaw_) {}
};

namespace puma_dwa_local_planner {
  class PumaDwaLocalPlanner : public nav_core::BaseLocalPlanner {
    public:
      PumaDwaLocalPlanner();
      PumaDwaLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

      ~PumaDwaLocalPlanner();

      void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      bool isGoalReached();

    private:
      /* Calcular costo de la trayectoria */
      double calculatePathCost(const std::vector<Position>& path);
      bool isValidPath(const std::vector<Position>& path);
      /* Simular trayectorias */
      std::vector<Position> simulatePath(double velocity, double steering_angle);
      /* Callback Odometry */
      void odometryCallback(const nav_msgs::Odometry& data);
      /* Normalizar angulo para giro */
      double normalizeAngle(double angle);
      /* Marker */
      visualization_msgs::Marker createDeleteAllMarker() const;
      visualization_msgs::Marker createPathMarker(const std::vector<Position>& path, int marker_id) const;
      /* Funciones auxiliares para calculo de ruta */
      double adjustSpeedBasedOnAngle(double angle);
      double calculateMaxAllowedVelocity();
      void evaluateAngle(double angle, double max_allowed_vel, double& best_cost, std::vector<Position>& best_path, geometry_msgs::Twist& cmd_vel, visualization_msgs::MarkerArray& marker_array, int& marker_id);
      double adjustVelocityForAcceleration(double target_velocity, double current_velocity);

      void getAdjustXYCostmap(double, double, int&, int&);
      /* Variables heredadas */
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_;
      tf2_ros::Buffer* tf_;
      bool initialized_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      geometry_msgs::PoseStamped goal_pose;
      bool reversing_;
      Position pos_start_reverse;

      /* Params */
      double max_velocity_, min_velocity_, max_steering_angle_;
      double reverse_limit_distance_;
      double min_turn_radius_, time_simulation_;
      double xy_goal_tolerance_, yaw_goal_tolerance_;
      double acceleration_x_, desacceleration_x_;
      double distance_for_desacceleration_;
      int steering_samples_, velocity_samples_;
      Position puma_;

      /* Factor cost */
      double factor_cost_deviation_, factor_cost_distance_goal_;

      std::string topic_odom_;
      ros::Subscriber odometry_puma;
      ros::Publisher trajectory_pub_;

  };

} // namespace puma_dwa_local_planner

#endif // PUMA_DWA_LOCAL_PLANNER_H