#include <pluginlib/class_list_macros.h>
#include "puma_global_planner/puma_global_planner.h"
#include <unordered_set>
#include <unordered_map>
extern "C" {
  #include "dubins.h"
}

PLUGINLIB_EXPORT_CLASS(puma_global_planner::PumaGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace puma_global_planner
{

  /* Constructor */
  PumaGlobalPlanner::PumaGlobalPlanner() {}
  PumaGlobalPlanner::PumaGlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    initialize(name, costmap_ros);
  }

  /* Inicializar el planificador */
  void PumaGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    if (!initialized_)
    {
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      ros::NodeHandle private_nh("~/" + name);

      loadRosParam(private_nh);
      
      dynamic_reconfigure::Server<puma_global_planner::PumaGlobalPlannerConfig> *dr_srv_ = new dynamic_reconfigure::Server<puma_global_planner::PumaGlobalPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<puma_global_planner::PumaGlobalPlannerConfig>::CallbackType cb = boost::bind(&PumaGlobalPlanner::reconfigureCB, this, _1, _2);
      dr_srv_->setCallback(cb);

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      plan_post_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan_post", 1);
      initialized_ = true;
    }
    else
    {
      ROS_WARN("Este planificador ya ha sido inicializado... no se hace nada");
    }
  }

  /* Cargar parámetros de ros */
  void PumaGlobalPlanner::loadRosParam(ros::NodeHandle &nh)
  {
    nh.param("resolution", resolution_, 0.5);
    nh.param("turning_radius", turning_radius_, 2.5);
    nh.param("meters_subsamples", meters_subsamples_, 7.0);
    nh.param("step_size_dubins", step_size_dubins_, 0.3);
  }

  void PumaGlobalPlanner::reconfigureCB(puma_global_planner::PumaGlobalPlannerConfig &config, uint32_t level)
{
  if (setup_ && config.restore_defaults)
  {
    config = default_config_;
    config.restore_defaults = false;
    ROS_INFO("%s: restored default config", ros::this_node::getName().c_str());
  }
  if (!setup_)
  {
    default_config_ = config;
    setup_ = true;
  }

  resolution_ = config.resolution;
  turning_radius_ = config.turning_radius; 
  step_size_dubins_ = config.step_size_dubins;  
  meters_subsamples_ = config.meters_subsamples;
}

  /* Generar plan */
  bool PumaGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
  {
    if (!initialized_)
    {
      ROS_ERROR("El planificador no ha sido inicializado, por favor inicializarlo antes de usarlo");
      return false;
    }

    plan.clear();
    Node start_node(start.pose.position.x, start.pose.position.y, tf::getYaw(start.pose.orientation), resolution_);
    Node goal_node(goal.pose.position.x, goal.pose.position.y, tf::getYaw(goal.pose.orientation), resolution_);

    ROS_INFO("Buscando camino desde (%f, %f) hasta (%f, %f)", start_node.x, start_node.y, goal_node.x, goal_node.y);
    try
    {
      auto t0 = std::chrono::high_resolution_clock::now();
      Node goal_reached = searchGoalAStar(start_node, goal_node);
      if (goal_reached == start_node)
      {
        ROS_WARN("No se ha encontrado un camino hacia el destino");
        return false;
      }
      nav_msgs::Path path = transformNodeReachedToPath(goal_reached);
      plan = path.poses;
      // Medicion de tiempo demorado 
      auto t1 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_seconds = t1 - t0;
      ROS_INFO("Tiempo de busqueda: %f segundos", elapsed_seconds.count());
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("Error al buscar el destino: %s", e.what());
      return false;
    }


    return true;
  }

  nav_msgs::Path PumaGlobalPlanner::transformNodeReachedToPath(const Node &node_goal_reached)
  {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    std::vector<std::shared_ptr<Node>> plan_nodes;
    auto path_node = std::make_shared<Node>(node_goal_reached);
    while (path_node)
    {
      plan_nodes.push_back(path_node);
      path_node = path_node->parent;
    }
    std::reverse(plan_nodes.begin(), plan_nodes.end());


    for (std::shared_ptr<Node> &node : plan_nodes)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map";
      pose.pose.position.x = node->x;
      pose.pose.position.y = node->y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(node->theta);
      path.poses.push_back(pose);
    }

    // PostProcesamiento de la trayectoria con dubin
    std::vector<Node> path_postprocessing = getDubinsCurvePath(plan_nodes);
    nav_msgs::Path path_post;
    path_post.header.frame_id = "map";
    path_post.header.stamp = ros::Time::now();

    for (Node &node : path_postprocessing)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map";
      pose.pose.position.x = node.x;
      pose.pose.position.y = node.y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.theta);
      path_post.poses.push_back(pose);
    }
    plan_post_pub_.publish(path_post);
    plan_pub_.publish(path);
    return path;
  }

  std::vector<Node> PumaGlobalPlanner::getDubinsCurvePath(std::vector<std::shared_ptr<Node>> &path) {
    std::vector<Node> path_postprocessing;
    int len_path = path.size();
    if (len_path < 2)
      return path_postprocessing; // no hay ruta para suavizar
    
    // Agregar el primer punto
    path_postprocessing.push_back(*path.front());
  
    // Definimos un intervalo de submuestreo (por ejemplo, cada 10 puntos)
    int i_prev = 0;
    // Iteramos sobre la ruta en bloques
    for (int i = 0; i < len_path; i++) {
      // Nos aseguramos de no salir del vector
      int i_end = i;
      if (i_end >= len_path)
      i_end = len_path - 1;
      
      std::shared_ptr<Node> node_prev = path[i_prev];
      std::shared_ptr<Node> node_end  = path[i_end];
      std::shared_ptr<Node> node_goal = path[len_path - 1];
      
      bool near_to_goal = node_goal->distance(*node_end) < meters_subsamples_;

      if (node_end->distance(*node_prev) > meters_subsamples_ || near_to_goal) {
        double q0[] = { node_prev->x, node_prev->y, node_prev->theta };
        
        if (near_to_goal)
          node_end = node_goal;

        double q1[] =  { node_end->x, node_end->y, node_end->theta };
        
        DubinsPath dubins_path;
        dubins_shortest_path(&dubins_path, q0, q1, turning_radius_);
        double length = dubins_path_length(&dubins_path);
        
        // Calculamos el número de muestras en base a la longitud y el step_size.
        int num_samples = std::max((int)std::ceil(length / step_size_dubins_), 2); // al menos 2 muestras
        // Se omite el primer punto ya que ya se agregó
        for (int j = 1; j < num_samples; j++) {
          double t = j * (length / (num_samples - 1)); // distribuir uniformemente entre 0 y length
          double q[3];
          dubins_path_sample(&dubins_path, t, q);
          Node node_dubins(q[0], q[1], q[2], resolution_);
          path_postprocessing.push_back(node_dubins);
        }
        if (near_to_goal) 
          break;
        
        i_prev = i_end;
      }
    }
  
    // Agregar el último punto de la ruta si no está incluido
    if (path_postprocessing.empty() || 
        (path_postprocessing.back().x != path.back()->x || path_postprocessing.back().y != path.back()->y))
    {
      path_postprocessing.push_back(*path.back());
    }
  
    return path_postprocessing;
  }

  /* Buscar el goal con A* */
  Node PumaGlobalPlanner::searchGoalAStar(const Node &start, const Node &goal)
  {
    std::priority_queue<Node> open_set;
    std::unordered_set<Node, NodeHash> open_set_hash;
    std::vector<Node> closed_set; /* Funcion hash */
    std::unordered_map<Node, double, NodeHash> known_costs;
    open_set.push(start);
    open_set_hash.insert(start);
    known_costs[start] = 0.0;

    while (!open_set.empty())
    {
      Node current = open_set.top();
      open_set.pop();

      if (nodeIsGoal(current, goal)) {
        current.x = goal.x;
        current.y = goal.y;
        current.theta = goal.theta;
        return current;
      }
      
      closed_set.push_back(current);

      for (int ix = -1; ix <= 1; ix++)
      {
        for (int iy = -1; iy <= 1; iy++)
        {
          if (ix == 0 && iy == 0)
            continue;

          Node neighbor = createNeighboor(current, goal, ix, iy);

          // Si el vecino ya está en closed, se salta.
          if (std::find(closed_set.begin(), closed_set.end(), neighbor) != closed_set.end())
            continue;
          
          
          if (open_set_hash.find(neighbor) == open_set_hash.end())
          {
            // Calcula el nuevo costo acumulado (g(n)) y la heurística (h(n))
            double tentative_g = current.prev_cost + current.distance(neighbor);
            double h = neighbor.distance(goal);
            double tentative_total_cost = tentative_g + h;

            
            if (known_costs.find(neighbor) == known_costs.end() || tentative_total_cost < known_costs[neighbor])
            {
              neighbor.prev_cost = tentative_g;
              neighbor.total_cost = tentative_total_cost;
              neighbor.parent = std::make_shared<Node>(current);
              known_costs[neighbor] = tentative_total_cost;
              open_set.push(neighbor);
              open_set_hash.insert(neighbor);
            }

          }
        }
      }
    }
    return start;
  }

  Node PumaGlobalPlanner::createNeighboor(const Node &current, const Node &goal, int ix, int iy)
  {
    double x = current.x + ix * resolution_;
    double y = current.y + iy * resolution_;
    double theta = atan2(y - current.y, x - current.x);
    Node neighbor(x, y, theta, resolution_);
    // Celdas
    neighbor.cell_x = current.cell_x + ix;
    neighbor.cell_y = current.cell_y + iy;
    return neighbor;
  }

  bool PumaGlobalPlanner::nodeIsGoal(const Node &node, const Node &goal)
  {
    return goal.x >= node.x_start && goal.x <= node.x_end && goal.y >= node.y_start && goal.y <= node.y_end;
  }

} // namespace puma_global_planner