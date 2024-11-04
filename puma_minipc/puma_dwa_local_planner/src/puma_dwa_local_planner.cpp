#include <pluginlib/class_list_macros.h>
#include "puma_dwa_local_planner/puma_dwa_local_planner.h"

PLUGINLIB_EXPORT_CLASS(puma_dwa_local_planner::PumaDwaLocalPlanner, nav_core::BaseLocalPlanner);

namespace puma_dwa_local_planner {
  /* Constructor */
  PumaDwaLocalPlanner::PumaDwaLocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

  PumaDwaLocalPlanner::PumaDwaLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false) {
    initialize(name, tf, costmap_ros);
  }

  PumaDwaLocalPlanner::~PumaDwaLocalPlanner() {}

  /* Inicializador */
  void PumaDwaLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      ros::NodeHandle private_nh("~/" + name);

      /* Cargar parametros */
      private_nh.param("max_velocity", max_velocity_, 1.0);
      private_nh.param("min_velocity", min_velocity_, 0.2);
      private_nh.param("max_steering_angle", max_steering_angle_, 0.52);
      private_nh.param("reverse_limit_distance", reverse_limit_distance_, 3.0);
      private_nh.param("time_simulation", time_simulation_, 2.0);
      private_nh.param("min_turn_radius", min_turn_radius_, 2.25);

      odometry_puma = private_nh.subscribe("odom", 2, &PumaDwaLocalPlanner::odometryCallback, this);
      trajectory_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("potential_trajectories", 1);

      initialized_ = true;
      ROS_INFO("PumaDwaLocalPlanner inicializado.");
    }
  }

  /* Callback odometry */
  void PumaDwaLocalPlanner::odometryCallback(const nav_msgs::Odometry& data) {
    puma_.x = data.pose.pose.position.x;
    puma_.y = data.pose.pose.position.y;
    puma_.yaw = tf::getYaw(data.pose.pose.orientation);
  }

  /* Definir plan */
  bool PumaDwaLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if(!initialized_){
      ROS_ERROR("El planificador PumaDwaLocalPlanner no ha sido inicializado.");
      return false;
    }
    global_plan_ = orig_global_plan;
    global_plan_running_ = orig_global_plan;
    ROS_INFO("Nuevo plan definido.");
    return true;
  }
  /* Calcular velocidades optima */
  bool PumaDwaLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if(!initialized_){
      ROS_ERROR("El planificador PumaDwaLocalPlanner no ha sido inicializado.");
      return false;
    }
    /* Crear variables */
    double best_cost = std::numeric_limits<double>::max();
    std::vector<Position> best_path;
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    /* Comprueba si existe informacion de la posicion del robot */
    if (std::isnan(puma_.x)) {
      ROS_WARN("Aun no se ha recibido informacion de la odometria.");
      return false;
    }

    /* Limpiar puntos ya completados */
    cleanGlobalPlan(global_plan_running_, puma_);
    int marker_id = 0;
    /* Ciclo para evaluar distintas velocidades */
    for (double vel = min_velocity_; vel < max_velocity_; vel += 0.1) {
      /* Fijar valor maximo */
      vel = std::min(vel, max_velocity_);
      /* Ciclo para evaluar distintos angulos de direcciones */
      for (double angle = -max_steering_angle_; angle < max_steering_angle_; angle += 0.1) {
        /* Fijar valor maximo */
        angle = std::min(angle, max_steering_angle_);

        auto path = simulatePath(vel, angle);
        double cost = calculatePathCost(path);

        if (cost < best_cost) {
          best_cost = cost;
          best_path = path;
          cmd_vel.linear.x = vel;
          cmd_vel.angular.z = angle;
        }
        // Crear un marcador para la trayectoria simulada
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";  // Ajustar según el marco de referencia
        marker.header.stamp = ros::Time::now();
        marker.ns = "trajectories";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;  // Grosor de la línea
        marker.color.a = 0.5;  // Transparencia
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        // Agregar puntos al marcador
        for (const auto& pos : path) {
          geometry_msgs::Point point;
          point.x = pos.x;
          point.y = pos.y;
          point.z = 0.0;
          marker.points.push_back(point);
        }

        marker_array.markers.push_back(marker);
      }
    }
    // Publicar todos los marcadores de trayectoria
    trajectory_pub_.publish(marker_array);

    return true;
  }

  /* Comprobar si se ha alcanzado el destino */
  bool PumaDwaLocalPlanner::isGoalReached() {
    if(!initialized_){
      ROS_ERROR("El planificador PumaDwaLocalPlanner no ha sido inicializado.");
      return false;
    }
    return true;
  }

  /* Generar la simulacion de la ruta */
  std::vector<Position> PumaDwaLocalPlanner::simulatePath(double velocity, double steering_angle) {
    std::vector<Position> path;
    path.reserve(static_cast<size_t>(time_simulation_ / 0.1) + 1);

    for (double t = 0 ; t < time_simulation_; t+=0.1) {
      t = std::min(t, time_simulation_);

      double factor = t / time_simulation_;
      double new_yaw = normalizeAngle(puma_.yaw + steering_angle * factor);
      double new_x = puma_.x + velocity * t * cos(new_yaw);
      double new_y = puma_.y + velocity * t * sin(new_yaw);

      path.emplace_back(new_x, new_y, new_yaw);
    }
    
    return path;
  }

  /* Calcular el costo asociado a la trayectoria  */
  double PumaDwaLocalPlanner::calculatePathCost(const std::vector<Position>& path) {
    double cost;

    /* Desviacion de la ruta */
    for (size_t i = 0; i < path.size(); ++i) {
      if ( i < global_plan_running_.size()) { // Comprobar si existe suficientes nodos de comparacion
        double deviation = std::hypot(path[i].x - global_plan_running_[i].pose.position.x, path[i].y - global_plan_running_[i].pose.position.y);
        cost += deviation;
      }
    }

    /* Costo por curvatura de la trayectoria */
    for (size_t i = 2; i < path.size(); ++i) {
      double angle_t1 = std::atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
      double angle_t2 = std::atan2(path[i - 1].y - path[i - 2].y, path[i - 1].x - path[i - 2].x);
      double angle_change = std::abs(angle_t1 - angle_t2);
      /* Normalizar para costo positivo -> 0 y M_PI*/
      if (angle_change > M_PI) {
          angle_change = 2 * M_PI - angle_change;
      }
      cost += angle_change;
    }

    return cost;
  }

  /* Limpiar ruta global */
  void PumaDwaLocalPlanner::cleanGlobalPlan(std::vector<geometry_msgs::PoseStamped>& global_plan, const Position& robot_position) {
    double min_distance = std::numeric_limits<double>::max();
    size_t closest_index = 0;

    for (size_t i = 0; i < global_plan.size(); ++i) {
        double dist = std::hypot(global_plan[i].pose.position.x - robot_position.x, global_plan[i].pose.position.y - robot_position.y);
        if (dist < min_distance) {
          min_distance = dist;
          closest_index = i;
        }
    }

    if (closest_index > 0) {
      global_plan.erase(global_plan.begin(),global_plan.begin() + closest_index);
    }

  }

  /* Normalizar para -M_PI A M_PI */
  double PumaDwaLocalPlanner::normalizeAngle(double angle){
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

} // namespace puma_dwa_local_planner