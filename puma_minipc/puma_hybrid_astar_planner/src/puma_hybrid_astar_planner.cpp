#include <pluginlib/class_list_macros.h>
#include "puma_hybrid_astar_planner/puma_hybrid_astar_planner.h"
#include <unordered_set>
#include <unordered_map>
#include <memory>

PLUGINLIB_EXPORT_CLASS(puma_hybrid_astar_planner::PumaHybridAStarPlanner, nav_core::BaseGlobalPlanner);

namespace puma_hybrid_astar_planner {
  /* Constructor */
  PumaHybridAStarPlanner::PumaHybridAStarPlanner(){}
  PumaHybridAStarPlanner::PumaHybridAStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
  }

  /* Inicializar */
  void PumaHybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if (!initialized_) {
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      ros::NodeHandle private_nh("~/"+name);

      // Cargar parámetros de rosparam
      loadRosParam(private_nh);
      initializePotentialMap();

      path_pub_ = private_nh.advertise<nav_msgs::Path>("global_sub_plan", 2);
      path_combined_pub_= private_nh.advertise<nav_msgs::Path>("global_plan", 2);
      potential_map_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential_map", 1);
      plan_pub_ = private_nh.advertise<nav_msgs::Path>(ns_plan_manager_ + std::string("/add"), 1);
      initialized_ = true;
    }
  }

  /* Cargar rosparam */
  void PumaHybridAStarPlanner::loadRosParam(ros::NodeHandle
  & nh){
    nh.param("step_size_meters", step_size_meters_, 0.3);
    nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.4);
    nh.param("factor_cost_distance", factor_cost_distance_, 10);
    nh.param("factor_cost_angle_curve", factor_cost_angle_curve_, 5);
    nh.param("division_curve", division_curve_, 3);
    nh.param("theta_limit", theta_limit_, M_PI / 5);
    nh.param("division_theta", division_theta_, 4);
    nh.param("factor_cost_angle_goal", factor_cost_angle_goal_, 2);
    nh.param("factor_cost_obstacle",factor_cost_obstacle_, 10);
    nh.param("factor_cost_unknown", factor_cost_unknown_, 1.0);
    nh.param("ns_waypoints_manager", ns_waypoints_manager_, std::string("/puma/navigation/waypoints"));
    nh.param("ns_plan_manager", ns_plan_manager_, std::string("/puma/navigation/plan"));
  }

  /* Inicializar potential map */
  void PumaHybridAStarPlanner::initializePotentialMap(){
    potential_map_.info.origin.position.x = costmap_->getOriginX();
    potential_map_.info.origin.position.y = costmap_->getOriginY();
    potential_map_.info.resolution = costmap_->getResolution();
    potential_map_.info.width = costmap_->getSizeInCellsX();
    potential_map_.info.height = costmap_->getSizeInCellsY(); 
    potential_map_.data.resize(potential_map_.info.width * potential_map_.info.height, -1); // -1 para celdas no revisadas
  }

  void PumaHybridAStarPlanner::publishPotentialMap() {
    potential_map_pub_.publish(potential_map_);
}
  /* Crear plan */
  bool PumaHybridAStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
      ROS_ERROR("No se ha inicializado correctamente Puma Hybrid A* planner.");
      return false;
    }
    
    // Esperar a recibir un mensaje del tópico de waypoints
    puma_msgs::WaypointNavConstPtr waypoints_msg = ros::topic::waitForMessage<puma_msgs::WaypointNav>(ns_waypoints_manager_ + std::string("/waypoints_info"), ros::Duration(10.0));
    if (!waypoints_msg) {
      ROS_WARN("No se recibió ningún mensaje de waypoints en el tiempo esperado.");
      return false;
    }
    waypoints_.clear();
    for (const puma_msgs::Waypoint& waypoint : waypoints_msg->waypoints) {
      Node new_pose = Node(waypoint.x, waypoint.y, waypoint.yaw);
      waypoints_.push_back(new_pose);
    }

    plan.clear();
    initializePotentialMap();
    
    nav_msgs::Path path_complete_;
    std::vector<geometry_msgs::PoseStamped> plan_complete;
    path_complete_.header.stamp = ros::Time::now();
    path_complete_.header.frame_id  = "map";
    ROS_INFO("Iniciando con el calculo de la ruta.");
    Node begin = Node(start.pose.position.x, start.pose.position.y, tf::getYaw(start.pose.orientation));
    ROS_INFO("Orientacion de inicio: %lf", begin.theta);
    for (const Node& point : waypoints_) {
      ROS_INFO("Busqueda hacia el destino -> x: %lf , y: %lf, theta: %lf", point.x, point.y, point.theta);
      /* Limpiar mapa de potencial */
      std::fill(potential_map_.data.begin(), potential_map_.data.end(), -1);
      
      std::vector<geometry_msgs::PoseStamped> result = getSubPlan(begin, point);
      if (result.size() == 0){
        ROS_WARN("Error a la hora de efectuar el plan");
        return false;
      }
      
      plan_complete.insert(plan_complete.end(), result.begin(), result.end());
      
      begin = point;
      path_complete_.poses = plan_complete;
      ROS_INFO("Punto de inicio (%.2f, %.2f) ", begin.x, begin.y);
      path_combined_pub_.publish(path_complete_);
    }
    plan = plan_complete;
    ROS_WARN("Plan creado exitosamente.");
    /* Subir plan generado */
    nav_msgs::Path path_generated_msg;
    path_generated_msg.header.stamp = ros::Time::now();
    path_generated_msg.header.frame_id  = "map"; 
    path_generated_msg.poses = plan;
    plan_pub_.publish(path_generated_msg);
    ros::Duration(1.0).sleep();
    return true;
  }

  std::vector<geometry_msgs::PoseStamped> PumaHybridAStarPlanner::getSubPlan(Node start, Node end) {
    auto start_node = std::make_shared<Node>(start);
    auto goal_node = std::make_shared<Node>(end);
    /* Obtencion del valor maximo al destino */
    dist_max_to_goal = std::abs(start_node->distance(*goal_node));
    getAdjustXYCostmap(*goal_node, goal_node->cell_x, goal_node->cell_y);
    /* Estructuras de datos */
    std::priority_queue<Node> open_set;
    std::unordered_set<Node, NodeHash> open_set_lookup;
    std::unordered_set<Node, NodeHash> closed_set;
    std::unordered_map<Node, double, NodeHash> known_costs; // Costo mínimo conocido para cada nodo

    /* Configuracion inicial */
    start_node->total_cost = 0.0;
    known_costs[*start_node] = 0.0;
    open_set.push(*start_node);
    open_set_lookup.insert(*start_node);
    int i = 0;
    /* Busqueda */
    while(!open_set.empty()) {
      auto current_node = std::make_shared<Node>(open_set.top());
      open_set.pop();

      /* Verificar si se llego al objetivo */
      if (goal_node->cell_x == current_node->cell_x && goal_node->cell_y == current_node->cell_y ) {
          ROS_INFO("Encontrado nodo objetivo. Transformando a ruta...");
          ROS_INFO("Encontrado nodo final -> x: %lf , y: %lf", current_node->x, current_node->y);

          nav_msgs::Path path_normal_msg;
          path_normal_msg.header.stamp = ros::Time::now();
          path_normal_msg.header.frame_id  = "map"; // Ajusta al frame de tu entorno

          std::vector<std::shared_ptr<Node>> plan_nodes;

          auto path_node = current_node;
          while (path_node) {
            plan_nodes.push_back(path_node);
            path_node = path_node->parent;
          }
          std::reverse(plan_nodes.begin(), plan_nodes.end());

          /* Generar el path normal */
          for (std::shared_ptr<Node>& node : plan_nodes) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose.position.x = node->x;
            pose.pose.position.y = node->y;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(node->theta);
            path_normal_msg.poses.push_back(pose);
            path_pub_.publish(path_normal_msg);
          }

          /* Reconstruir plan final */
          
          path_pub_.publish(path_normal_msg);
          return path_normal_msg.poses;
      }
      closed_set.insert(*current_node);

      /* Expandir nodo */
      std::vector<std::shared_ptr<Node>> new_nodes = expandNode(*current_node, *goal_node);
      for (auto& new_node : new_nodes) {
        /* Si el nodo está en el conjunto cerrado, se ignora */
        if (closed_set.find(*new_node) != closed_set.end()) {
          continue;
        }

        /* Se realiza calculo solo si el nodo no esta en el conjunto de revisados*/
        if (open_set_lookup.find(*new_node) == open_set_lookup.end()) {
          updatePotentialMap(*current_node);

          double new_cost = current_node->total_cost + current_node->distance(*new_node);
          /* Verifica y actualiza si el costo es menor o el nodo es nuevo en known_costs */
          if (known_costs.find(*new_node) == known_costs.end() || new_cost < known_costs[*new_node]) {
            /* Actualizar costos y relacion de nodos */
            updateCostNode(new_node, new_cost, goal_node, current_node);
            /* Añade el nodo a los conjuntos de datos */
            known_costs[*new_node] = new_cost;
            open_set.push(*new_node);
            open_set_lookup.insert(*new_node);
          }
        }
      }
      i += 1;
      if (i % 10 == 0) { // Publica cada 10 iteraciones
          publishPotentialMap();
      }
    }
    return std::vector<geometry_msgs::PoseStamped>();
  }

  /* Actualizar mapa potencial */
  void PumaHybridAStarPlanner::updatePotentialMap(const Node& current_node ) {
    int cell_x, cell_y;
    getAdjustXYCostmap(current_node, cell_x, cell_y);
    
    if (cell_x >= 0 && cell_y >= 0 && 
        cell_x < potential_map_.info.width && 
        cell_y < potential_map_.info.height) {
        potential_map_.data[cell_y * potential_map_.info.width + cell_x] = 0; // Marcando como revisado
    }
  }

  /* Actualizar costo de los nodos */
  void PumaHybridAStarPlanner::updateCostNode(std::shared_ptr<Node>& new_node, double new_cost,  std::shared_ptr<Node>& goal_node, std::shared_ptr<Node>& current_node) {
    /* Costo asociado a total anterior*/
    new_node->cost = new_cost;

    /* Costo heuristica new_node -> goal */
    double normalized_distance_cost = (new_node->distance(*goal_node)/ dist_max_to_goal) * factor_cost_distance_;

    /* Costo asociado a cambio de angulo entre nodos cercanos */
    double normalized_angle_curve = std::abs(current_node->theta - new_node->theta) * factor_cost_angle_curve_;

    /* Calculo de direccion entre new_node -> goal */
    double dist_to_goal = std::abs(new_node->distance(*goal_node));
    double dx = goal_node->x - new_node->x;
    double dy = goal_node->y - new_node->y;
    double angle_diff_to_goal = normalizeAngle(atan2(dy,dx) - goal_node->theta);

    /* Calculo costo asociado de la direccion de new_node -> goal */
    double goal_alignment_cost = std::abs(angle_diff_to_goal) * factor_cost_angle_goal_;

    /* Calculo de costo asociado a obstaculos */
    unsigned char cell_cost = costmap_->getCost(new_node->cell_x, new_node->cell_y);
    double cell_cost_with_factor = cell_cost == costmap_2d::NO_INFORMATION ? cell_cost * factor_cost_unknown_ : cell_cost;
    double normalized_obstacle_cost = (cell_cost_with_factor / 255.0) * factor_cost_obstacle_;

    /* Total de costos */
    new_node->total_cost = new_node->cost + normalized_distance_cost + normalized_obstacle_cost + normalized_angle_curve + goal_alignment_cost ;
  }

  /* Expandir nodo */
  std::vector<std::shared_ptr<Node>> PumaHybridAStarPlanner::expandNode(const Node& current_node, const Node& goal) {
    std::vector<std::shared_ptr<Node>> new_nodes;
    double distance_to_goal = current_node.distance(goal);

    if (distance_to_goal <= xy_goal_tolerance_) {
      ROS_INFO_THROTTLE(5,"Aplicando threshold con nodo final -> x: %lf , y: %lf",goal.x, goal.y);
      auto new_node = std::make_shared<Node>(goal.x, goal.y, goal.theta);
      getAdjustXYCostmap(*new_node, new_node->cell_x, new_node->cell_y);
      new_node->parent = std::make_shared<Node>(current_node);
      new_nodes.push_back(new_node);
    } else {
      /* Generar nodos para avanzar */
      /* Ciclo de distintos angulos*/
      for (double angle = -theta_limit_; angle < theta_limit_; angle+= theta_limit_ / division_theta_) {
        double new_theta = current_node.theta + angle;
        double new_x = current_node.x + step_size_meters_ * cos(new_theta);
        double new_y = current_node.y + step_size_meters_ * sin(new_theta);
        auto new_node = std::make_shared<Node>(new_x, new_y, new_theta);
        getAdjustXYCostmap(*new_node, new_node->cell_x, new_node->cell_y);

        // Verificar validez del nuevo nodo
        if (isValidNode(*new_node)) {
          new_node->parent = std::make_shared<Node>(current_node); 
          new_nodes.push_back(new_node);
        }
      }
    }
  
    return new_nodes;
  }
  /* Normalizar angulos entre pi y -pi */
  double PumaHybridAStarPlanner::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  /* Generar curvas Dubin */
  std::vector<std::shared_ptr<Node>> PumaHybridAStarPlanner::generateDubinsPath(const Node& start, const Node& end) {
    std::vector<std::shared_ptr<Node>> path;
    std::shared_ptr<Node> previous_node = std::make_shared<Node>(start);

    double delta_dist = std::hypot(end.x - start.x, end.y - start.y);
    double delta_theta = normalizeAngle(end.theta - start.theta); 

    if (division_curve_ <= 0) {
      ROS_ERROR_THROTTLE(10,"Parametro division_curve invalido. Proporcionar un valor mayor a 0.");
      return path;
    }

    /* division_curve_ determina la cantidad de pasos */
    for (double i = 1; i < division_curve_; i+= 1) {
      double scale = i/division_curve_;
      double new_theta = normalizeAngle(start.theta + delta_theta * scale);
      double new_x = start.x + delta_dist * scale * cos(new_theta);
      double new_y = start.y + delta_dist * scale * sin(new_theta);

      auto new_node = std::make_shared<Node>(new_x, new_y, new_theta);
      new_node->parent = previous_node;

      path.emplace_back(new_node);
      previous_node = new_node;
    }
    
    return path;
  }

  /* Validar si es un nodo correcto */
  bool PumaHybridAStarPlanner::isValidNode(const Node& node) {
    int cell_x, cell_y;
    getAdjustXYCostmap(node, cell_x, cell_y);

    /* Verifica limites del mapa de costos */
    if (cell_x < 0 || cell_y < 0 || 
        cell_x >= costmap_->getSizeInCellsX()|| 
        cell_y >= costmap_->getSizeInCellsY()) {
        ROS_INFO_THROTTLE(2," Nodo evaluado invalido, se pasa del limite del costmap.");
        return false;
    }
    /* Verifica colision */
    if (costmap_->getCost(cell_x, cell_y) == costmap_2d::LETHAL_OBSTACLE) {
      ROS_WARN_THROTTLE(2,"Nodo evaluado invalido, ocurre una colision.");
      return false;
    }
    return true;
  }

  /* Calcular valor de celdas ajustada */
  void PumaHybridAStarPlanner::getAdjustXYCostmap(const Node& node, int& cell_x, int& cell_y) {
    /* Calcular posiciones ajustadas al origen del mapa */
    double adjusted_x = node.x - costmap_->getOriginX();
    double adjusted_y = node.y - costmap_->getOriginY();

    /* Convertir las posiciones a indice de celdas */
    cell_x = static_cast<int>(adjusted_x / costmap_->getResolution());
    cell_y = static_cast<int>(adjusted_y / costmap_->getResolution());
  }

};