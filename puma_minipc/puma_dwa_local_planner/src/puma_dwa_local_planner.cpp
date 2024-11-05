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
      private_nh.param("desacceleration_x", desacceleration_x_, 0.3);
      private_nh.param("acceleration_x", acceleration_x_, 0.2);
      private_nh.param("distance_for_desacceleration", distance_for_desacceleration_, 4.0);
      private_nh.param("reverse_limit_distance", reverse_limit_distance_, 3.0);
      private_nh.param("time_simulation", time_simulation_, 2.0);
      private_nh.param<std::string>("topic_odom", topic_odom_, "odom");
      private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.4);
      private_nh.param("steering_samples", steering_samples_, 6);
      private_nh.param("velocity_samples", velocity_samples_, 4);
      private_nh.param("factor_cost_deviation",factor_cost_deviation_ , 2.0);
      private_nh.param("factor_cost_distance_goal", factor_cost_distance_goal_, 15.0);

      odometry_puma = private_nh.subscribe("/"+topic_odom_, 2, &PumaDwaLocalPlanner::odometryCallback, this);
      trajectory_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("potential_trajectories", 1);

      initialized_ = true;
      reversing_ = false;
      ROS_INFO("PumaDwaLocalPlanner inicializado.");
    }
  }

  /* Callback odometry */
  void PumaDwaLocalPlanner::odometryCallback(const nav_msgs::Odometry& data) {
    puma_.x = data.pose.pose.position.x;
    puma_.y = data.pose.pose.position.y;
    puma_.yaw = tf::getYaw(data.pose.pose.orientation);
    puma_.vel_x = data.twist.twist.linear.x;
  }

  /* Definir plan */
  bool PumaDwaLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if(!initialized_){
      ROS_ERROR("El planificador PumaDwaLocalPlanner no ha sido inicializado.");
      return false;
    }
    global_plan_.clear();
    global_plan_ = orig_global_plan;
    goal_pose = global_plan_.back();
    ROS_INFO("Nuevo plan definido.");
    return true;
  }

  /* Calcular velocidades optima */
  bool PumaDwaLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if(!initialized_){
      ROS_ERROR("El planificador PumaDwaLocalPlanner no ha sido inicializado.");
      return false;
    }

    /* Comprueba si existe informacion de la posicion del robot */
    if (std::isnan(puma_.x)) {
      ROS_WARN("Aun no se ha recibido informacion de la odometria. Revisar topico %s. -- PumaDwaLocalPlanner", topic_odom_.c_str());
      return false;
    }

    /* Control de estado en reversa*/
    if (reversing_){
      double distance_run = std::hypot(pos_start_reverse.x - puma_.x, pos_start_reverse.y - puma_.y);
      if (distance_run < reverse_limit_distance_) {
        cmd_vel.linear.x = -min_velocity_;  // Velocidad hacia atrás
        cmd_vel.angular.z = 0;
        ROS_WARN_THROTTLE(1, "Retrocediendo para intentar evitar obstáculo -- PumaDwaLocalPlanner.");
        return true;
      } else {
        cmd_vel.linear.x = 0;
        reversing_ = false;
        return false;
      }
    }

    /* Crear variables */
    double best_cost = std::numeric_limits<double>::max();
    std::vector<Position> best_path;
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(createDeleteAllMarker());

    /* Fijar valor maximo */
    int marker_id = 0;
    double angle_steps = max_steering_angle_ * 2 / steering_samples_;

    /* Ajustar velocidad respecto a distancia objetivo */
    double max_allowed_vel = calculateMaxAllowedVelocity();

    /* Ciclo para evaluar distintos angulos de direcciones */
    for (double angle = -max_steering_angle_; angle < max_steering_angle_; angle += angle_steps) {
      evaluateAngle(angle, max_allowed_vel, best_cost, best_path, cmd_vel, marker_array, marker_id);
    }
    /* Crear la linea recta */
    evaluateAngle(0, max_allowed_vel, best_cost, best_path, cmd_vel, marker_array, marker_id);

    /* Publicar todos los marcadores de trayectoria */ 
    trajectory_pub_.publish(marker_array);
    if (best_cost == std::numeric_limits<double>::max()) {
      ROS_WARN_THROTTLE(5,"No se ha encontrado ninguna ruta posible -- PumaDwaLocalPlanner.");
      cmd_vel.linear.x, cmd_vel.angular.z = 0;
      pos_start_reverse = puma_;
      reversing_ = true;
    }
    else 
      /* Corregir velocidad segun acceleracion */
      cmd_vel.linear.x = adjustVelocityForAcceleration(cmd_vel.linear.x, puma_.vel_x);

    return true;
  }

  /* Evaluar camino posible segun velocidad - direccion y actualizar cmd_vel */
  void PumaDwaLocalPlanner::evaluateAngle(double angle, double max_allowed_vel, double& best_cost, std::vector<Position>& best_path, geometry_msgs::Twist& cmd_vel, visualization_msgs::MarkerArray& marker_array, int& marker_id) {
    double adjusted_velocity = adjustSpeedBasedOnAngle(angle);
    adjusted_velocity = std::max(std::min(adjusted_velocity,max_allowed_vel), min_velocity_);

    auto path = simulatePath(adjusted_velocity, angle);
    double cost = calculatePathCost(path);

    if (cost < best_cost && isValidPath(path)) {
        best_cost = cost;
        best_path = path;
        cmd_vel.linear.x = adjusted_velocity;
        cmd_vel.angular.z = angle;
    }

    // Agregar el marcador del camino evaluado
    marker_array.markers.push_back(createPathMarker(path, marker_id++));
  }

  /* Comprobar ruta valida sin colision*/
  bool PumaDwaLocalPlanner::isValidPath(const std::vector<Position>& path) {
    for (const auto& pos : path) {
      // int cell_x, cell_y;
      // getAdjustXYCostmap(pos, cell_x, cell_y);
      // unsigned char cost = costmap_->getCost(cell_x, cell_y);
      // if (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      //   return false;
      std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
      for (const auto& point : footprint) {
        int cell_x, cell_y;
        double footprint_x = pos.x + point.x*cos(pos.yaw) - point.y*sin(pos.yaw);
        double footprint_y = pos.y + point.x*sin(pos.yaw) + point.y*cos(pos.yaw);
        getAdjustXYCostmap(footprint_x, footprint_y, cell_x, cell_y);
        unsigned char cost = costmap_->getCost(cell_x, cell_y);
        if (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
          return false;
      }
    }
    return true;
  }

  /* Limpiar rutas del marcador*/
  visualization_msgs::Marker PumaDwaLocalPlanner::createDeleteAllMarker() const {
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    return delete_marker;
  }

  /* Craear rutas en marcador */
  visualization_msgs::Marker PumaDwaLocalPlanner::createPathMarker(const std::vector<Position>& path, int marker_id) const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";  // Ajustar según el marco de referencia
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectories";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;  // Grosor de la línea
    marker.color.a = 1.0;  // Transparencia
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

    return marker;
}

  /* Comprobar si se ha alcanzado el destino */
  bool PumaDwaLocalPlanner::isGoalReached() {
    if(!initialized_){
      ROS_ERROR("El planificador PumaDwaLocalPlanner no ha sido inicializado.");
      return false;
    }
    
    if (global_plan_.empty()) {
      ROS_WARN("El plan global esta vacio, no se puede comprobar el destino.");
      return false;
    }

    /* Comprobar distancia dentro del rango */
    double goal_x = goal_pose.pose.position.x;
    double goal_y = goal_pose.pose.position.y;
    double distance_to_goal = std::hypot(puma_.x - goal_x, puma_.y - goal_y);

    if (distance_to_goal <= xy_goal_tolerance_) {
      ROS_INFO("El robot ha alcanzado el destino dentro de la tolerancia %f metros.", xy_goal_tolerance_);
      return true;
    }

    return false;
  }

  /* Generar la simulacion de la ruta */
  std::vector<Position> PumaDwaLocalPlanner::simulatePath(double velocity, double steering_angle) {
    std::vector<Position> path;
    path.reserve(static_cast<size_t>(time_simulation_ / 0.1) + 1);

    double delta_t = 0.1;
    for (double t = 0 ; t < time_simulation_; t+=delta_t) {
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

    double min_distance = std::numeric_limits<double>::max();
    size_t closest_index = 0;
    for (size_t i = 0; i < global_plan_.size(); ++i) {
        double dist = std::hypot(global_plan_[i].pose.position.x - puma_.x, global_plan_[i].pose.position.y - puma_.y);
        if (dist < min_distance) {
          min_distance = dist;
          closest_index = i;
        }
    }
    if (min_distance == std::numeric_limits<double>::max()) {
      ROS_WARN("Erro al estimar distancia entre la ruta planteada y la global");
    }

    /* Desviacion de la ruta */
    for (size_t i = 0; i < path.size() && (closest_index + i) < global_plan_.size(); ++i) {
      double deviation = std::hypot(path[i].x - global_plan_[closest_index + i].pose.position.x, path[i].y - global_plan_[closest_index + i].pose.position.y);
      cost += deviation * factor_cost_deviation_;
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

    /* Costo por distancia al objetivo */
    double distance_to_goal = std::hypot(path.back().x - goal_pose.pose.position.x, 
                                        path.back().y - goal_pose.pose.position.y);
    cost += distance_to_goal * factor_cost_distance_goal_;

    return cost;
  }

  /* Ajustar velocidad segun angulo de giro */
  double PumaDwaLocalPlanner::adjustSpeedBasedOnAngle(double angle) {
    double angle_ratio = std::abs(angle) / max_steering_angle_;

    double adjusted_velocity = max_velocity_ - (max_velocity_ - min_velocity_) * angle_ratio;
    return std::max(adjusted_velocity, min_velocity_);
  }

  /* Normalizar para -M_PI A M_PI */
  double PumaDwaLocalPlanner::normalizeAngle(double angle){
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  /* Calcular maxima velocidad permitida */
  double PumaDwaLocalPlanner::calculateMaxAllowedVelocity()  {
    double distance_to_goal = std::hypot(puma_.x - goal_pose.pose.position.x, puma_.y - goal_pose.pose.position.y);
    if (distance_to_goal < distance_for_desacceleration_) {
        return std::max(min_velocity_, max_velocity_ - desacceleration_x_ * 
                        (distance_for_desacceleration_ - distance_to_goal) / max_velocity_);
    } else {
        return max_velocity_;
    }
  }
  
  /* Ajustar velocidad segun la aceleracion */
  double PumaDwaLocalPlanner::adjustVelocityForAcceleration(double target_velocity, double current_velocity) {
    if (current_velocity < target_velocity) {
        return std::min(current_velocity + acceleration_x_, target_velocity);
    } else {
        return std::max(current_velocity - acceleration_x_, target_velocity);
    }
  }
  /* Calcular valor de celdas ajustada */
  void PumaDwaLocalPlanner::getAdjustXYCostmap(double pos_x, double pos_y, int& cell_x, int& cell_y) {
    /* Calcular posiciones ajustadas al origen del mapa */
    double adjusted_x = pos_x - costmap_->getOriginX();
    double adjusted_y = pos_y - costmap_->getOriginY();

    /* Convertir las posiciones a indice de celdas */
    cell_x = static_cast<int>(adjusted_x / costmap_->getResolution());
    cell_y = static_cast<int>(adjusted_y / costmap_->getResolution());
  }

} // namespace puma_dwa_local_planner