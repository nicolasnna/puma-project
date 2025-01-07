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
      private_nh.param("factor_cost_deviation",factor_cost_deviation_ , 2.0);
      private_nh.param("factor_cost_distance_goal", factor_cost_distance_goal_, 15.0);
      private_nh.param("factor_cost_angle_to_plan", factor_cost_angle_to_plan_, 4.0);
      private_nh.param("factor_cost_obstacle", factor_cost_obstacle_, 3.0);

      odometry_puma = private_nh.subscribe("/"+topic_odom_, 2, &PumaDwaLocalPlanner::odometryCallback, this);
      trajectory_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("potential_trajectories", 1);
      path_local_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan",1);

      initialized_ = true;
      reversing_ = false;
      ROS_INFO("PumaDwaLocalPlanner inicializado.");
    }
  }

  /* Callback odometry */
  void PumaDwaLocalPlanner::odometryCallback(const nav_msgs::Odometry& data) {
    double x = data.pose.pose.position.x;
    double y = data.pose.pose.position.y;
    double yaw = tf::getYaw(data.pose.pose.orientation);
    double vel_x = data.twist.twist.linear.x;

    // Verificar si los valores son válidos
    if (std::isnan(x) || std::isnan(y) || std::isnan(yaw) || std::isnan(vel_x)) {
        ROS_WARN("Se recibió odometría con valores NaN. Verifica el sensor.");
        return;
    }

    if (std::isinf(x) || std::isinf(y) || std::isinf(yaw) || std::isinf(vel_x)) {
        ROS_WARN("Se recibió odometría con valores infinitos. Verifica el sensor.");
        return;
    }

    // Asignar valores si son válidos
    puma_.x = x;
    puma_.y = y;
    puma_.yaw = yaw;
    puma_.vel_x = vel_x;
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
      ROS_WARN("Aun no se ha recibido informacion de la odometria. esperando topico %s. -- PumaDwaLocalPlanner", topic_odom_.c_str());
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
      cmd_vel.linear.x = cmd_vel.angular.z = 0;
      pos_start_reverse = puma_;
      reversing_ = true;
    }
    else 
      /* Corregir velocidad segun acceleracion */
      cmd_vel.linear.x = adjustVelocityForAcceleration(cmd_vel.linear.x, puma_.vel_x);
    /* Publicar plan a seguir */
    publishLocalPath(best_path);

    return true;
  }

  /* Evaluar camino posible segun velocidad - direccion y actualizar cmd_vel */
  void PumaDwaLocalPlanner::evaluateAngle(double angle, double max_allowed_vel, double& best_cost, std::vector<Position>& best_path, geometry_msgs::Twist& cmd_vel, visualization_msgs::MarkerArray& marker_array, int& marker_id) {
    double adjusted_velocity = adjustSpeedBasedOnAngle(angle);
    adjusted_velocity = std::max(std::min(adjusted_velocity,max_allowed_vel), min_velocity_);

    auto path = simulatePath(adjusted_velocity, angle);
    if (isValidPath(path)) {
      double cost = calculatePathCost(path);

      if (cost < best_cost) {
          best_cost = cost;
          best_path = path;
          cmd_vel.linear.x = adjusted_velocity;
          cmd_vel.angular.z = angle;
      }

      // Agregar el marcador del camino evaluado
      marker_array.markers.push_back(createPathMarker(path, marker_id++));
    }
  }

  /* Comprobar ruta valida sin colision*/
  bool PumaDwaLocalPlanner::isValidPath(const std::vector<Position>& path) {
    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    for (const auto& pos : path) {
      for (const auto& point : footprint) {
        int cell_x, cell_y;
        double footprint_x = pos.x + point.x*cos(pos.yaw) - point.y*sin(pos.yaw);
        double footprint_y = pos.y + point.x*sin(pos.yaw) + point.y*cos(pos.yaw);
        getAdjustXYCostmap(footprint_x, footprint_y, cell_x, cell_y);
        if (cell_x < 0 || cell_y < 0 || 
          cell_x >= costmap_->getSizeInCellsX() || 
          cell_y >= costmap_->getSizeInCellsY()) {
          ROS_INFO_THROTTLE(3," Nodo evaluado invalido, se pasa del limite del costmap.");
          return false;
        } 
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
  /* Publicar ruta a seguir */
  void PumaDwaLocalPlanner::publishLocalPath(std::vector<Position> path) {
    nav_msgs::Path path_local;
    path_local.header.frame_id = "map";
    path_local.header.stamp = ros::Time::now();
    for (auto& pose_path : path) {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map";
      pose.pose.position.x = pose_path.x;
      pose.pose.position.y = pose_path.y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_path.yaw);
      path_local.poses.push_back(pose);
    }
    path_local_pub_.publish(path_local);
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

    if (distance_to_goal < 3.0) {
    
      /* Comprobar estado del destino */
      int cell_x, cell_y;
      getAdjustXYCostmap(goal_x, goal_y, cell_x, cell_y);
      if (cell_x < 0 || cell_y < 0 || 
        cell_x >= costmap_->getSizeInCellsX() || 
        cell_y >= costmap_->getSizeInCellsY()) {
        ROS_WARN("Celda de comprobacion de costos invalido, sale de los limites de local cost.");
        ROS_WARN("Goalx %.2f Goaly %.2f CellX %d Celly %d", goal_x, goal_y, cell_x, cell_y);
        return false; 
      } 
      unsigned char cost = costmap_->getCost(cell_x, cell_y);

      if (cost == costmap_2d::LETHAL_OBSTACLE ) {
        ROS_WARN("El destino se encuentra en un obstaculo. Finalizando navegación...");
        return true;
      }

      if (cost >= (unsigned char) 80U && cost < (unsigned char) 253U) {
        ROS_WARN("El destino se encuentra cercano a un obstaculo. Finalizando navegación...");
        return true;
      }
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

    double last_x = path.back().x;
    double last_y = path.back().y;
    double last_yaw = path.back().yaw;
    double min_distance_end = std::numeric_limits<double>::max();
    double min_distance_begin = std::numeric_limits<double>::max();
    size_t closest_index_end = 0;
    size_t closest_index_begin = 0;
    for (size_t i = 0; i < global_plan_.size(); ++i) {
      double dist_end = std::hypot(global_plan_[i].pose.position.x - last_x, global_plan_[i].pose.position.y - last_y);
      if (dist_end < min_distance_end) {
        min_distance_end = dist_end;
        closest_index_end = i;
      }
      double dist_begin = std::hypot(global_plan_[i].pose.position.x - puma_.x, global_plan_[i].pose.position.y - puma_.y);
      if (dist_begin < min_distance_begin) {
        min_distance_begin = dist_begin;
        closest_index_begin = i;
      }
    }

    if (min_distance_end == std::numeric_limits<double>::max()) {
      ROS_WARN("Error al estimar distancia entre la ruta planteada y la global");
      return std::numeric_limits<double>::max();
    }
  
    /* Desviacion de la ruta */
    int divitionsPath = path.size() / (closest_index_end-closest_index_begin);
    for (size_t i = closest_index_begin; i < closest_index_end; i++) {
      int indexPath = (i - closest_index_begin)* divitionsPath;
      double deviation = std::hypot(path[indexPath].x - global_plan_[i].pose.position.x, path[indexPath].y - global_plan_[i].pose.position.y);
      cost += deviation * factor_cost_deviation_;
    }
    double deviation_final = std::hypot(last_x- global_plan_[closest_index_end].pose.position.x, last_y - global_plan_[closest_index_end].pose.position.y);
    cost += deviation_final * factor_cost_deviation_;

    /* Costo asociado de diferencia de angulo respecto al plan */
    double normalized_cost_angle = std::abs(last_yaw - tf::getYaw(global_plan_[closest_index_end].pose.orientation));
    cost += normalized_cost_angle + factor_cost_angle_to_plan_;

    /* Costo por distancia al objetivo */
    double distance_to_goal = std::hypot(last_x - goal_pose.pose.position.x, 
                                        last_y - goal_pose.pose.position.y);
    cost += distance_to_goal * factor_cost_distance_goal_;

    /* Costo por obstaculo */
    for (size_t i = 0; i < path.size(); i++) {
      int cell_x, cell_y;
      getAdjustXYCostmap(path[i].x, path[i].y, cell_x, cell_y);
      unsigned char cell_cost = costmap_->getCost(cell_x, cell_y);
      double normalized_obstacle_cost = (cell_cost / 255.0 ) * factor_cost_obstacle_;
      cost += normalized_obstacle_cost;
    }

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