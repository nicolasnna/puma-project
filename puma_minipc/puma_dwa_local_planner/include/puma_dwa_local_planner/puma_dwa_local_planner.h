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
  Position() : x(NAN), y(NAN), yaw(NAN) {}
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
      /* Simular trayectorias */
      std::vector<Position> simulatePath(double velocity, double steering_angle);
      /* Callback Odometry */
      void odometryCallback(const nav_msgs::Odometry& data);
      /* Normalizar angulo para giro */
      double normalizeAngle(double angle);
      /* Limpiar planificador global */
      void cleanGlobalPlan(std::vector<geometry_msgs::PoseStamped>& global_plan, const Position& robot_position);
      /* Variables heredadas */
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_;
      tf2_ros::Buffer* tf_;
      bool initialized_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      std::vector<geometry_msgs::PoseStamped> global_plan_running_;

      /* Params */
      double max_velocity_, min_velocity_, max_steering_angle_;
      double reverse_limit_distance_;
      double min_turn_radius_;
      double time_simulation_;
      Position puma_;

      ros::Subscriber odometry_puma;
      ros::Publisher trajectory_pub_;

  };

} // namespace puma_dwa_local_planner

#endif // PUMA_DWA_LOCAL_PLANNER_H