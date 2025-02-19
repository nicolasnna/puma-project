#include <pluginlib/class_list_macros.h>
#include <puma_local_planner/puma_local_planner.h>

PLUGINLIB_EXPORT_CLASS(puma_local_planner::PumaLocalPlanner, nav_core::BaseLocalPlanner)

namespace puma_local_planner{
  /* Constructor */
  PumaLocalPlanner::PumaLocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}
  PumaLocalPlanner::PumaLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false) {
    initialize(name, tf, costmap_ros);
  }
  /* Destructor */
  PumaLocalPlanner::~PumaLocalPlanner() {}

  /* Inicializador */
  void PumaLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      tf_ = tf;
      ros::NodeHandle private_nh("~/" + name);

      /* Cargar parámetros */
      loadParameters(private_nh);

      dynamic_reconfigure::Server<puma_local_planner::PumaLocalPlannerConfig> *dr_srv_ = new dynamic_reconfigure::Server<puma_local_planner::PumaLocalPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<puma_local_planner::PumaLocalPlannerConfig>::CallbackType cb = boost::bind(&PumaLocalPlanner::reconfigureCB, this, _1, _2);
      dr_srv_->setCallback(cb);

      /* Pubs y Subs */
      odometry_sub_ = private_nh.subscribe("/" + topic_odom_, 1, &PumaLocalPlanner::odometryCb, this);
      path_global_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      path_local_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      trajectory_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("potential_trajectories", 1);

      initialized_ = true;
      reversing_mode_ = false;
    }else{
      ROS_WARN("Este planificador ya ha sido inicializado, no se hará nada.");
    }
  }

  /* Cargar parametros */
  void PumaLocalPlanner::loadParameters(ros::NodeHandle& nh){
    nh.param<std::string>("topic_odom", topic_odom_, "odom");
    nh.param<int>("max_index_path_compare", max_index_path_compare_, 10);
    nh.param<double>("turning_radius", turning_radius_, 2.5);
    nh.param<double>("max_velocity", max_velocity_, 1.0);
    nh.param<double>("min_velocity", min_velocity_, 0.1);
    nh.param<double>("steering_rads_limit", steering_rads_limit_, 0.698);
    nh.param<int>("steering_samples", steering_samples_, 10);
    nh.param<int>("velocity_samples", velocity_samples_, 1);
    nh.param<double>("max_acceleration", max_acceleration_, 0.5);
    nh.param<double>("max_deceleration", max_deceleration_, 0.5);
    nh.param<double>("distance_for_deceleration", distance_for_deceleration_, 1.0);
    nh.param<double>("time_simulation", time_simulation_, 1.0);
    nh.param<double>("time_step", time_step_, 0.1);
    nh.param<double>("xy_goal_tolerance", xy_goal_tolerance_, 0.5);
    nh.param<double>("factor_velocity", factor_velocity_, 1.0);
  }

  void PumaLocalPlanner::reconfigureCB(puma_local_planner::PumaLocalPlannerConfig &config, uint32_t level){
    if (setup_ && config.restore_defaults) {
      config = default_config_;
      config.restore_defaults = false;
      ROS_INFO("%s: restored default config", ros::this_node::getName().c_str());
    }
    if (!setup_) {
      default_config_ = config;
      setup_ = true;
    }

    max_index_path_compare_ = config.max_index_path_compare;
    turning_radius_ = config.turning_radius;
    max_velocity_ = config.max_velocity;
    min_velocity_ = config.min_velocity;
    steering_rads_limit_ = config.steering_rads_limit;
    max_acceleration_ = config.max_acceleration;
    max_deceleration_ = config.max_deceleration;
    distance_for_deceleration_ = config.distance_for_deceleration;
    time_simulation_ = config.time_simulation;
    time_step_ = config.time_step;
    steering_samples_ = config.steering_samples;
    velocity_samples_ = config.velocity_samples;
    xy_goal_tolerance_ = config.xy_goal_tolerance;
  }

  /* Callback odometry */
  void PumaLocalPlanner::odometryCb(const nav_msgs::Odometry::ConstPtr& msg){
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = tf::getYaw(msg->pose.pose.orientation);
    double vel_x = msg->twist.twist.linear.x;

    /* Verificar valores válidos en casos de error*/
    if(std::isnan(x) || std::isnan(y) || std::isnan(yaw) || std::isnan(vel_x)){
      ROS_WARN("Valores de odometría no válidos.");
      return;
    }

    if(std::isinf(x) || std::isinf(y) || std::isinf(yaw) || std::isinf(vel_x)){
      ROS_WARN("Valores de odometría no válidos.");
      return;
    }

    puma_.x = x;
    puma_.y = y;
    puma_.yaw = yaw;
    puma_.vel_x = vel_x;
  }

  bool PumaLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
    if (!initialized_){
      ROS_ERROR("Este planificador no ha sido inicializado, por favor llame a la función initialize().");
      return false;
    }
    global_plan_.clear();
    global_plan_ = plan;

    geometry_msgs::PoseStamped goal = global_plan_.back();
    goal_.x = goal.pose.position.x;
    goal_.y = goal.pose.position.y;
    goal_.yaw = tf::getYaw(goal.pose.orientation);

    ROS_INFO("Plan recibido con %d puntos.", (int)global_plan_.size());
    return true;
  }

  void PumaLocalPlanner::updatePlan(){
    if(global_plan_.empty())
      return;

    double min_distance = std::numeric_limits<double>::max();
    size_t closest_index = 0;
    size_t max_index = std::min(global_plan_.size(), size_t(max_index_path_compare_));

    /* Obtener distancia más cercana */
    for (size_t i = 0; i < max_index; i++){
      double dx = global_plan_[i].pose.position.x - puma_.x;
      double dy = global_plan_[i].pose.position.y - puma_.y;
      double distance = sqrt(dx*dx + dy*dy);

      if(distance < min_distance){
        min_distance = distance;
        closest_index = i;
      }
    }
    /* Limpiar plan recorrido */
    if (closest_index > 0 )
      global_plan_.erase(global_plan_.begin(), global_plan_.begin() + closest_index);

    /* Publicar plan global */
    nav_msgs::Path global_path;
    global_path.header.frame_id = "map";
    global_path.header.stamp = ros::Time::now();
    global_path.poses = global_plan_;
    path_global_pub_.publish(global_path);
  }

  double PumaLocalPlanner::calculateMaxAllowedVelocity(){
    double distance_to_goal = std::hypot(goal_.x - puma_.x, goal_.y - puma_.y);
    double filter_vel = std::max(min_velocity_, max_velocity_ - max_deceleration_ * (distance_for_deceleration_ - distance_to_goal) / max_velocity_);

    if (distance_to_goal < distance_for_deceleration_)
      return filter_vel;
    
    return max_velocity_;
  }

  double PumaLocalPlanner::normalizeAngle(const double angle){
    double angle_norm = angle;
    while(angle_norm > M_PI) angle_norm -= 2.0 * M_PI;
    while(angle_norm < -M_PI) angle_norm += 2.0 * M_PI;

    return angle_norm;
  }


  /* Calcular comandos de velocidad */
  bool PumaLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    if (!initialized_)
      return false;
    
    /* Actualizar plan */
    updatePlan();

    /* Comprueba si existe informacion de posicion del robot */
    if (std::isnan(puma_.x) || std::isnan(puma_.y) || std::isnan(puma_.yaw)){
      ROS_WARN("Aun no se ha recibido informacion de la odometria. esperando topico %s. -- PumaLocalPlanner", topic_odom_.c_str());
      return false;
    }

    std::vector<Position> best_path = simulatePaths();    
    if (best_path.empty()){
      ROS_WARN("No se encontraron caminos validos.");
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      return false;
    }

    Position best_position = best_path.back();
    cmd_vel.linear.x = best_position.vel_x;
    cmd_vel.angular.z = best_position.ang_z;

    publishLocalPath(best_path);
    return true;
  }

  std::vector<Position> PumaLocalPlanner::simulatePaths(){
    /* Variable de salida */
    std::vector<Position> best_path;
    double best_cost = std::numeric_limits<double>::max();
    /* Marker para visualizacion */
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    markers.markers.push_back(createDeleteAllMarker());

    /* Parametros a simular */
    double max_allowed_velocity = calculateMaxAllowedVelocity();
    double angle_steps = steering_rads_limit_ * 2 / steering_samples_;
    double vel_steps = (max_allowed_velocity - min_velocity_) / velocity_samples_;

    for (int iv = 0; iv < velocity_samples_; iv++){
      double vel = min_velocity_ + iv * vel_steps;
      for (int ia = 0; ia < steering_samples_; ia++) {
        double angle = -steering_rads_limit_ + ia * angle_steps;
        std::vector<Position> path = generatePath(vel, angle);

        if (path.empty())
          continue;
        
        double cost = calculateCost(path);
        if (cost < best_cost){
          best_cost = cost;
          best_path = path;
        }
        markers.markers.push_back(createPathMarker(path, marker_id++));
      }

      std::vector<Position> path = generatePath(vel, 0);

      if (path.empty())
        continue;

      double cost = calculateCost(path);
      if (cost < best_cost){
        best_cost = cost;
        best_path = path;
      }
      markers.markers.push_back(createPathMarker(path, marker_id++));
    }

    trajectory_pub_.publish(markers);
    return best_path;
  }

  std::vector<Position> PumaLocalPlanner::generatePath(double vel, double angle){
    std::vector<Position> path;
    path.push_back(puma_);
    
    for (double t = 0; t < time_simulation_ ; t+=time_step_){
      t = std::min(t, time_simulation_);
      double factor = t / time_simulation_;
      double new_yaw = normalizeAngle(puma_.yaw + angle * factor);
      double x = puma_.x + vel * cos(new_yaw) * t;
      double y = puma_.y + vel * sin(new_yaw) * t;

      Position pos = Position(x, y, new_yaw, vel, angle);
      if (!isValidPose(pos))
        return std::vector<Position>();

      path.push_back(pos);
    }
    return path;
  }

  double PumaLocalPlanner::calculateCost(std::vector<Position>& path) {
    double cost_acumulative = 0.0;
    
    size_t max_index = std::min(global_plan_.size(), size_t(max_index_path_compare_));

    for (auto position : path) {
      double min_distance = std::numeric_limits<double>::max();
      size_t closest_index = 0;
      
      for (size_t i = 0; i < max_index; i++){
        double dx = global_plan_[i].pose.position.x - path.back().x;
        double dy = global_plan_[i].pose.position.y - path.back().y;
        double distance = sqrt(dx*dx + dy*dy);
  
        if(distance < min_distance){
          min_distance = distance;
          closest_index = i;
        }
      }
      cost_acumulative += min_distance;
    }

    return cost_acumulative / (1 + factor_velocity_ * path.back().vel_x);
  }

  bool PumaLocalPlanner::isValidPose(const Position& pos) {
    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    for (const auto& point : footprint) {
      int cell_x, cell_y;
      double footprint_x = pos.x + point.x*cos(pos.yaw) - point.y*sin(pos.yaw);
      double footprint_y = pos.y + point.x*sin(pos.yaw) + point.y*cos(pos.yaw);
      getAdjustXYCostmap(footprint_x, footprint_y, cell_x, cell_y);
      if (cell_x < 0 || cell_y < 0 || 
        cell_x >= costmap_->getSizeInCellsX() || 
        cell_y >= costmap_->getSizeInCellsY()) {
        return false;
      } 
      unsigned char cost = costmap_->getCost(cell_x, cell_y);
      if (cost > 30U)
        return false;
    }
    return true;
  }

  void PumaLocalPlanner::getAdjustXYCostmap(double pos_x, double pos_y, int& cell_x, int& cell_y) {
    /* Calcular posiciones ajustadas al origen del mapa */
    double adjusted_x = pos_x - costmap_->getOriginX();
    double adjusted_y = pos_y - costmap_->getOriginY();

    /* Convertir las posiciones a indice de celdas */
    cell_x = static_cast<int>(adjusted_x / costmap_->getResolution());
    cell_y = static_cast<int>(adjusted_y / costmap_->getResolution());
  }

  /* Limpiar rutas del marcador*/
  visualization_msgs::Marker PumaLocalPlanner::createDeleteAllMarker() const {
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    return delete_marker;
  }


  visualization_msgs::Marker PumaLocalPlanner::createPathMarker(const std::vector<Position>& path, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";  // Ajustar según el marco de referencia
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectories";
    marker.id = id;
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
  void PumaLocalPlanner::publishLocalPath(std::vector<Position> path) {
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

  bool PumaLocalPlanner::isGoalReached(){
    if (!initialized_)
      return false;

    if(global_plan_.empty())
      return true;

    if (validateGoalReached(goal_.x, goal_.y, puma_.x, puma_.y))
      return true;

    return false;
  }

  bool PumaLocalPlanner::validateGoalReached(double goal_x, double goal_y, double current_x, double current_y) {
    double distance_to_goal = std::hypot(goal_x - current_x, goal_y - current_y);
    if (distance_to_goal <= xy_goal_tolerance_) {
      ROS_INFO("El robot ha alcanzado el destino dentro de la tolerancia %f metros.", xy_goal_tolerance_);
      return true;
    }

    if (distance_to_goal < 3.0) {
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

      if (cost > 10U && cost <= 254U) {
        ROS_WARN("El destino se encuentra en un obstaculo. Objetivo completado...");
        return true;
      }
    }
    return false;
  }

}