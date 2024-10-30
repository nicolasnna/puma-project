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

      origin_x_costmap_ = costmap_->getOriginX();
      origin_y_costmap_ = costmap_->getOriginY();
      resolution_costmap_ = costmap_->getResolution();
      cells_x_costmap_ = costmap_->getSizeInCellsX();
      cells_y_costmap_ = costmap_->getSizeInCellsY();

      initializePotentialMap();

      path_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 2);
      path_dubin_pub_= private_nh.advertise<nav_msgs::Path>("global_dubin_plan", 2);
      potential_map_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential_map", 1);
      
      initialized_ = true;
    }
  }
  /* Cargar rosparam */
  void PumaHybridAStarPlanner::loadRosParam(ros::NodeHandle
  & nh){
    nh.param("max_step_size", max_step_size_, 2.3);
    nh.param("min_step_size", min_step_size_, 0.6);
    nh.param("orientation_tolerance", orientation_tolerance_, 0.5);
    nh.param("xy_tolerance", xy_tolerance_, 0.5);
    nh.param("factor_cost_distance", factor_cost_distance_, 10);
    nh.param("factor_cost_angle", factor_cost_angle_, 5);
    nh.param("division_curve", division_curve_, 3);
    nh.param("enable_dubin", enable_dubin_, true);
    nh.param("max_steering_angle", max_steering_angle_, M_PI / 5);
    nh.param("wheel_base", wheel_base_, 1.15);
    nh.param("final_threshold", final_threshold_, 10.0);
    nh.param("division_theta_", division_theta_, 4);
    nh.param("factor_cost_angle_goal", factor_cost_angle_goal_, 2);
    nh.param("division_steps", division_steps_,4);
    nh.param("divisor_factor_increment_step", divisor_factor_increment_step, 20.0);
    nh.param("distance_threshold_step", distance_threshold_step, 25.0);
    nh.param("max_iteration_search", max_iteration_search, 10000);
    nh.param("factor_cost_obstacle",factor_cost_obstacle_, 10);
  }

  /* Inicializar potential map */
  void PumaHybridAStarPlanner::initializePotentialMap(){
      potential_map_.info.resolution = costmap_->getResolution();
      potential_map_.info.width = costmap_->getSizeInCellsX();
      potential_map_.info.height = costmap_->getSizeInCellsY();
      potential_map_.info.origin.position.x = costmap_->getOriginX();
      potential_map_.info.origin.position.y = costmap_->getOriginY();
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
    plan.clear();
    /* Limpiar mapa de potencial */
    std::fill(potential_map_.data.begin(), potential_map_.data.end(), -1);

    if (enable_dubin_) {
      ROS_INFO("Usando curvas dubin...");
    } else {
      ROS_INFO("Ejecutando sin suavizado de curvas dubin...");
    }
    
    ROS_INFO("Iniciando con el calculo de la ruta.");
    ROS_INFO("Busqueda hacia el destino -> x: %lf , y: %lf", goal.pose.position.x, goal.pose.position.y);
    /* Crear nodos iniciales y finales */
    auto start_node = std::make_shared<Node>(start.pose.position.x, start.pose.position.y, tf::getYaw(start.pose.orientation));
    auto goal_node = std::make_shared<Node>(goal.pose.position.x, goal.pose.position.y, tf::getYaw(goal.pose.orientation));

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
    
    int iteration_count = 0;
    while(!open_set.empty() && iteration_count < max_iteration_search) {
      auto current_node = std::make_shared<Node>(open_set.top());
      open_set.pop();

      /* Verificar si se llego al objetivo */
      if ( std::hypot(std::abs(current_node->x - goal_node->x), std::abs(current_node->y - goal_node->y)) <= xy_tolerance_
        &&  (std::abs(current_node->theta - goal_node->theta) <= orientation_tolerance_)) {
          ROS_INFO("Encontrado nodo objetivo. Transformando a ruta...");
          ROS_INFO("Encontrado nodo final -> x: %lf , y: %lf", current_node->x, current_node->y);

          nav_msgs::Path path_normal_msg;
          nav_msgs::Path path_dubin_msg;
          path_normal_msg.header.stamp = path_dubin_msg.header.stamp = ros::Time::now();
          path_normal_msg.header.frame_id = path_dubin_msg.header.frame_id = "map"; // Ajusta al frame de tu entorno

          std::vector<std::shared_ptr<Node>> plan_nodes;
          std::vector<std::shared_ptr<Node>> plan_dubins;

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

          /* Generar curva dubins en nodos intermedios */
          if (enable_dubin_) {
            std::shared_ptr<Node> previus_node = plan_nodes.front();
            plan_dubins.push_back(previus_node);
            for (auto it = plan_nodes.begin() + 1; it != plan_nodes.end(); ++it) {
              std::shared_ptr<Node> node_end = *it;
              std::vector<std::shared_ptr<Node>> nodes_extra = generateDubinsPath(*previus_node, *node_end);
              if (nodes_extra.empty()) {
                ROS_WARN("Error al generar las curvas dubins");
                return false;
              }
              for (std::shared_ptr<Node>& node : nodes_extra) {
                plan_dubins.push_back(node);
              }
              previus_node = *it;
            }
            for (std::shared_ptr<Node>& node : plan_dubins) {
              geometry_msgs::PoseStamped pose;
              pose.header.stamp = ros::Time::now();
              pose.header.frame_id = "map";
              pose.pose.position.x = node->x;
              pose.pose.position.y = node->y;
              pose.pose.orientation = tf::createQuaternionMsgFromYaw(node->theta);
              path_dubin_msg.poses.push_back(pose);
              path_dubin_pub_.publish(path_dubin_msg);
            }
          }

          /* Reconstruir plan final */
          plan = enable_dubin_? path_dubin_msg.poses : path_normal_msg.poses;
          
          path_pub_.publish(path_normal_msg);
          path_dubin_pub_.publish(path_dubin_msg);
          return true;
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
      iteration_count += 1;
      i += 1;
      if (i % 10 == 0) { // Publica cada 10 iteraciones
          publishPotentialMap();
      }
    }

    ROS_WARN("No se encontro una ruta al objetivo.");
    return false;
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
    new_node->heuristic = pow(new_node->distance(*goal_node), factor_cost_distance_);

    /* Costo asociado a cambio de angulo entre nodos cercanos */
    double angle_diff_cost = pow(std::abs(current_node->theta - new_node->theta)* 4, factor_cost_angle_);

    /* Calculo de direccion entre new_node -> goal */
    double dist_to_goal = std::abs(new_node->distance(*goal_node));
    double dx = goal_node->x - new_node->x;
    double dy = goal_node->y - new_node->y;
    double angle_diff_to_goal = normalizeAngle(atan2(dy,dx) - goal_node->theta);

    /* Cambio de factor linear segun distancia a goal */
    int factor_lineal_proximity = dist_to_goal < 8 ? 10 : 4;

    /* Calculo costo asociado de la direccion de new_node -> goal */
    double angle_goal_cost = pow(std::abs(angle_diff_to_goal)*factor_lineal_proximity, factor_cost_angle_goal_);

    // /* Costo de proximidad a obstáculos */
    // unsigned int cell_x, cell_y;
    // costmap_->worldToMap(new_node->x, new_node->y, cell_x, cell_y);
    // // Definir el radio para analizar proximidad a obstáculos (en celdas)
    // double proximity_radius = 5.0; // por ejemplo, 5 metros
    // unsigned int cells_radius = static_cast<unsigned int>(proximity_radius / costmap_->getResolution());
    // double obstacle_proximity_cost = 0;
    // for (int ix = -cells_radius; ix <= cells_radius; ++ix) {
    //     for (int iy = -cells_radius; iy <= cells_radius; ++iy) {
    //         unsigned int nx = cell_x + ix;
    //         unsigned int ny = cell_y + iy;

    //         if (nx >= costmap_->getSizeInCellsX() || ny >= costmap_->getSizeInCellsY()) {
    //             continue; // Salir si está fuera del mapa
    //         }

    //         unsigned char cell_cost = costmap_->getCost(nx, ny);
    //         if (cell_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    //             // Calcular distancia desde el nodo al obstáculo
    //             double distance = costmap_->getResolution() * sqrt(ix * ix + iy * iy);
    //             // Penalización inversamente proporcional a la distancia
    //             if (distance < proximity_radius) {
    //                 obstacle_proximity_cost += pow((proximity_radius - distance) / proximity_radius, factor_cost_obstacle_) * (cell_cost / 255.0) * 10;
    //             }
    //         }
    //     }
    // }

    /* Costo de proximidad a obstáculos */
    unsigned int cell_x, cell_y;
    costmap_->worldToMap(new_node->x, new_node->y, cell_x, cell_y);
    unsigned char obstacle_cost = costmap_->getCost(cell_x, cell_y);

    double obstacle_proximity_cost = 0.0;
    if (obstacle_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        // Penalización cuadrática basada en el valor del costmap, más alta cuanto mayor es el riesgo
        obstacle_proximity_cost = pow(obstacle_cost / 255.0 * 10, factor_cost_obstacle_) ;
    }

    new_node->total_cost = new_node->cost + new_node->heuristic + angle_diff_cost + angle_goal_cost + obstacle_proximity_cost;
  }

  /* Expandir nodo */
  std::vector<std::shared_ptr<Node>> PumaHybridAStarPlanner::expandNode(const Node& current_node, const Node& goal) {
    std::vector<std::shared_ptr<Node>> new_nodes;
    double distance_to_goal = current_node.distance(goal);

    double max_step;
    if (divisor_factor_increment_step <= 0) {
      ROS_WARN_ONCE("Parametro -divisor_factor_increment_step- invalido. Debe ser mayor a 0.");
      max_step = max_step_size_;
    } else {
      max_step = distance_to_goal < distance_threshold_step ? max_step_size_ : max_step_size_ * (distance_to_goal / divisor_factor_increment_step);
    }

    if (distance_to_goal < final_threshold_) {
      ROS_INFO("Aplicando threshold con nodo final -> x: %lf , y: %lf",goal.x, goal.y);
      auto final_node = std::make_shared<Node>(goal.x, goal.y, goal.theta);
      final_node->parent = std::make_shared<Node>(current_node);
      new_nodes.push_back(final_node);
    } else {
      /* Generar nodos para avanzar */
      /* Ciclo de distintos steps*/
      double delta_steps = max_step - min_step_size_;
      for (double step = min_step_size_; step < max_step; step += delta_steps / division_steps_) {
        step = step > max_step ? max_step : step;
        theta_limit_ = calculateThetaLimit(step);
        /* Ciclo de distintos angulos*/
        for (double angle = -theta_limit_; angle < theta_limit_; angle+= theta_limit_ / division_theta_) {
          double new_theta = current_node.theta + angle;
          double new_x = current_node.x + step * cos(new_theta);
          double new_y = current_node.y + step * sin(new_theta);

          auto new_node = std::make_shared<Node>(new_x, new_y, new_theta);
          // Verificar validez del nuevo nodo
          if (isValidNode(*new_node)) {
            new_node->parent = std::make_shared<Node>(current_node);  // Usar puntero compartido para asignar al padre
            new_nodes.push_back(new_node);
          }
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
      ROS_ERROR("Parametro division_curve invalido. Proporcionar un valor mayor a 0.");
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
        cell_x >= cells_x_costmap_ || 
        cell_y >= cells_y_costmap_) {
        ROS_INFO("Se pasa del limite del costmap.");
        return false;
    }
    /* Verifica colision */
    if (costmap_->getCost(cell_x, cell_y) >= costmap_2d::LETHAL_OBSTACLE) {
      ROS_INFO("Ocurre una colisión.");
      return false;
    }
    return true;
  }


  /* Calcular valor de celdas ajustada */
  void PumaHybridAStarPlanner::getAdjustXYCostmap(const Node& node, int& cell_x, int& cell_y) {
    /* Calcular posiciones ajustadas al origen del mapa */
    double adjusted_x = node.x - origin_x_costmap_;
    double adjusted_y = node.y - origin_y_costmap_;

    /* Convertir las posiciones a indice de celdas */
    cell_x = static_cast<int>(adjusted_x / resolution_costmap_);
    cell_y = static_cast<int>(adjusted_y / resolution_costmap_);
  }


  /* Calcular theta limit segun wheel base, max sterring y step_size */
  double PumaHybridAStarPlanner::calculateThetaLimit(double step){
    double min_turn_radius = wheel_base_ / tan(max_steering_angle_);
    return step / min_turn_radius;
  }
};