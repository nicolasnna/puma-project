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
      private_nh.param("max_step_size", max_step_size_, 2.3);
      private_nh.param("min_step_size", min_step_size_, 0.6);
      private_nh.param("orientation_tolerance", orientation_tolerance_, 0.5);
      private_nh.param("xy_tolerance", xy_tolerance_, 0.5);
      private_nh.param("factor_cost_distance", factor_cost_distance_, 10);
      private_nh.param("factor_cost_angle", factor_cost_angle_, 5);
      private_nh.param("division_curve", division_curve_, 3);
      private_nh.param("enable_dubin", enable_dubin_, true);
      private_nh.param("max_steering_angle", max_steering_angle_, M_PI / 5);
      private_nh.param("wheel_base", wheel_base_, 1.15);
      private_nh.param("final_threshold", final_threshold_, 10.0);
      private_nh.param("divisions", divisions_, 4);

      origin_x_costmap_ = costmap_->getOriginX();
      origin_y_costmap_ = costmap_->getOriginY();
      resolution_costmap_ = costmap_->getResolution();
      cells_x_costmap_ = costmap_->getSizeInCellsX();
      cells_y_costmap_ = costmap_->getSizeInCellsY();

      initializePotentialMap();

      path_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 2);
      potential_map_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential_map", 1);
      
      initialized_ = true;
    }
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

    ROS_INFO("Iniciando con el calculo de la ruta.");
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
    while(!open_set.empty()) {
      auto current_node = std::make_shared<Node>(open_set.top());
      open_set.pop();

      /* Verificar si se llego al objetivo */
      if ( std::hypot(std::abs(current_node->x - goal_node->x), std::abs(current_node->y - goal_node->y)) <= xy_tolerance_
        &&  (std::abs(current_node->theta - goal_node->theta) <= orientation_tolerance_)) {
          ROS_INFO("Encontrado nodo objetivo. Transformando a ruta...");
          ROS_INFO("Encontrado x: %lf - y: %lf --- Objetivo x: %lf - y: %lf", current_node->x, current_node->y, goal_node->x, goal_node->y);

          nav_msgs::Path path_msg;
          path_msg.header.stamp = ros::Time::now();
          path_msg.header.frame_id = "map"; // Ajusta al frame de tu entorno
          // Reconstruir el plan
          auto path_node = current_node;
          while (path_node) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose.position.x = path_node->x;
            pose.pose.position.y = path_node->y;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(path_node->theta);
            plan.push_back(pose);
            path_msg.poses.push_back(pose);
            path_node = path_node->parent;
            path_pub_.publish(path_msg);
          }
          std::reverse(path_msg.poses.begin(), path_msg.poses.end());
          std::reverse(plan.begin(), plan.end());
          path_pub_.publish(path_msg);
          return true;
      }

      closed_set.insert(*current_node);
      /* Calcular Step Size dinamicamente */
      calculateDynamicStepSizeMeters(*current_node, *goal_node);

      if (enable_dubin_) {
        /* Expandir nodo con dubin */
        std::vector<std::vector<std::shared_ptr<Node>>> vector_new_nodes = expandNodeDubins(*current_node, *goal_node);
        updatePotentialMap(*current_node);
        for (auto& vector_new_node : vector_new_nodes) {
          auto previus_node = std::make_shared<Node>(*current_node);
          for (auto& new_node : vector_new_node) {
            /* Si el nodo esta en el conjunto cerrado, se ignora */
            if (closed_set.find(*new_node) != closed_set.end()) {
              continue;
            }

            /* Si el nodo no esta en el conjunto de revisados */
            if (open_set_lookup.find(*new_node) == open_set_lookup.end()) {
              updatePotentialMap(*new_node);

              double new_cost = previus_node->total_cost + previus_node->distance(*new_node);
              /* Verifica y actualiza si el costo es menor o el nodo es nuevo en known_costs */
              if (known_costs.find(*new_node) == known_costs.end() || new_cost < known_costs[*new_node]) {
                /* Actualizar costos y relacion de nodos */
                updateCostNode(new_node, new_cost, goal_node, previus_node);
                /* Añade el nodo a los conjuntos de datos */
                known_costs[*new_node] = new_cost;
                open_set.push(*new_node);
                open_set_lookup.insert(*new_node);
                previus_node = new_node;
              } else {
                break;
              }
            }
          }
        }
      } else {
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
      }

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
    new_node->cost = new_cost;
    new_node->heuristic = pow(new_node->distance(*goal_node), factor_cost_distance_);

    double angle_diff = pow(std::abs(new_node->theta - goal_node->theta)* 4, factor_cost_angle_);
    new_node->total_cost = new_node->cost + new_node->heuristic + angle_diff;
    
  }

  /* Expandir nodo */
  std::vector<std::shared_ptr<Node>> PumaHybridAStarPlanner::expandNode(const Node& current_node, const Node& goal) {
    std::vector<std::shared_ptr<Node>> new_nodes;
    double distance_to_goal = current_node.distance(goal);

    if (distance_to_goal < final_threshold_) {
      auto final_node = std::make_shared<Node>(goal.x, goal.y, goal.theta);
      final_node->parent = std::make_shared<Node>(current_node);
      new_nodes.push_back(final_node);
    } else {
      /* Generar nodos para avanzar */
      for (double angle = -theta_limit_; angle <= theta_limit_; angle+= theta_limit_ / divisions_) {
        double new_x = current_node.x + step_size_meters_ * cos(current_node.theta + angle);
        double new_y = current_node.y + step_size_meters_ * sin(current_node.theta + angle);
        double new_theta = current_node.theta + angle;

        auto new_node = std::make_shared<Node>(new_x, new_y, new_theta);
        // Verificar validez del nuevo nodo
        if (isValidNode(*new_node)) {
          new_node->parent = std::make_shared<Node>(current_node);  // Usar puntero compartido para asignar al padre
          new_nodes.push_back(new_node);
        }
      }
    
    }
    return new_nodes;
  }

  /* Expandir nodo en curvas de dubin */
  std::vector<std::vector<std::shared_ptr<Node>>> PumaHybridAStarPlanner::expandNodeDubins(const Node& current_node, const Node& goal) {
    std::vector<std::vector<std::shared_ptr<Node>>> new_node_groups;

    double distance_to_goal = current_node.distance(goal);

    // if (distance_to_goal < final_threshold_) {
    //   /* Curvas Dubins */
    //   std::vector<std::shared_ptr<Node>> trajectory = generateDubinsPath(current_node, goal.theta, true, distance_to_goal);
    //   new_node_groups.push_back(trajectory);

    // } else {
    /* Generar nodos para avanzar */
    for (double angle = -theta_limit_; angle <= theta_limit_; angle += theta_limit_ / divisions_) {
        /* Curvas Dubins */
        std::vector<std::shared_ptr<Node>> trajectory = generateDubinsPath(current_node, angle, false, 0);

        /* Verificar */
        if (isValidTrajectory(trajectory)) {
            new_node_groups.push_back(trajectory); // Añadir la trayectoria válida al grupo
        }
      }
    // }
    return new_node_groups; // Devolver el vector de vectores

  }

  /* Generar curvas Dubin */
  std::vector<std::shared_ptr<Node>> PumaHybridAStarPlanner::generateDubinsPath(const Node& start, double steer_angle, bool final_node, double dist_final) {
    std::vector<std::shared_ptr<Node>> path;
    std::shared_ptr<Node> previous_node = std::make_shared<Node>(start);

    if (final_node) {
      double step = dist_final / division_curve_;
      for (double t = 0; t < dist_final; t += step) {
        double new_x = start.x + t * cos(steer_angle);
        double new_y = start.y + t * sin(steer_angle);

        auto new_node = std::make_shared<Node>(new_x, new_y, steer_angle);
        new_node->parent = previous_node;

        path.emplace_back(new_node);
        previous_node = new_node;
      }

    } else {
      double theta = start.theta + steer_angle;
      double step = step_size_meters_ / division_curve_;
      for (double i = 0; i < step_size_meters_; i+= step) {
        double new_x = start.x + i * cos(theta);
        double new_y = start.y + i * sin(theta);

        auto new_node = std::make_shared<Node>(new_x, new_y, theta);
        new_node->parent = previous_node;

        path.emplace_back(new_node);
        previous_node = new_node;
      }
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

  /* Validar trajectoria , arreglo de nodos */
  bool PumaHybridAStarPlanner::isValidTrajectory(std::vector<std::shared_ptr<Node>>& trajectory) {
    for (auto& point: trajectory) {
      /* Si algun nodo es invalido */
      if (!isValidNode(*point)) {
        return false;
      }
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

  /* Calcular dinamicamente los step size meters */
  void PumaHybridAStarPlanner::calculateDynamicStepSizeMeters(const Node& current_node, const Node& goal_node) {
    double distance_to_goal = current_node.distance(goal_node);

    /* Rangos */
    if (distance_to_goal > 15) {
      step_size_meters_ =  max_step_size_;
    } else if (distance_to_goal > 8 ) {
      step_size_meters_ = (max_step_size_ + min_step_size_) / 2 ;
    } else {
      step_size_meters_ = min_step_size_;
    }
    /* Calcular nuevo theta limit */
    calculateThetaLimit();
  }

  /* Calcular theta limit segun wheel base, max sterring y step_size */
  void PumaHybridAStarPlanner::calculateThetaLimit(){
    double min_turn_radius = wheel_base_ / tan(max_steering_angle_);
    theta_limit_ = step_size_meters_ / min_turn_radius;
  }
};