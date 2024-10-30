#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <cmath>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <dynamic_reconfigure/server.h>


using std::string;

struct Node {
  double x;
  double y;
  double theta;
  std::shared_ptr<Node> parent;
  double cost;
  double heuristic;
  double total_cost;
  /* Constructor */
  Node(double x_, double y_, double theta_)
      : x(x_), y(y_), theta(theta_), parent(nullptr), cost(0), heuristic(0), total_cost(0) {}
  /* Operador para el ordenamiento en estructuras como priority_queue */
  bool operator<(const Node& other) const {
    return total_cost > other.total_cost; // Para que el menor costo tenga mayor prioridad
  }
  /* Metodo para calcular la distancia entre nodos */
  double distance(const Node& other) const {
    return std::hypot(x - other.x, y - other.y);
  }
  /* Operador de igualdad */
  bool operator==(const Node& other) const {
    return (x == other.x) && (y == other.y) && (theta == other.theta);
  }
};

/* Funcion hash */
struct NodeHash {
  std::size_t operator()(const Node& node) const {
    /* Combinar x, y, theta en hash */
    std::hash<double> hash_fn;
    std::size_t hash_x = hash_fn(node.x);
    std::size_t hash_y = hash_fn(node.y);
    std::size_t hash_theta = hash_fn(node.theta);

    return hash_x ^ (hash_y << 1) ^ (hash_theta << 2);
  }
};

#ifndef PUMA_HYBRID_ASTAR_PLANNER_CPP
#define PUMA_HYBRID_ASTAR_PLANNER_CPP

namespace puma_hybrid_astar_planner {

class PumaHybridAStarPlanner : public nav_core::BaseGlobalPlanner {
  public:
    PumaHybridAStarPlanner();
    PumaHybridAStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan
                  );
    /* Expandir nodos */
    std::vector<std::shared_ptr<Node>> expandNode(const Node& current_node, const Node& goal);
    /* Manejo de potential map */
    void publishPotentialMap();
    void initializePotentialMap();
    void updatePotentialMap(const Node& current_node);
    void loadRosParam(ros::NodeHandle& nh);

    void updateCostNode(std::shared_ptr<Node>& new_node, double new_cost,  std::shared_ptr<Node>& goal_node, std::shared_ptr<Node>& current_node);

    double calculateThetaLimit(double step);

  private:
    bool isValidNode(const Node& node);
    std::vector<std::shared_ptr<Node>> generateDubinsPath(const Node& start, const Node& end);
    void getAdjustXYCostmap(const Node& node, int& cell_x, int& cell_y);
    double normalizeAngle(double angle);

    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    bool initialized_ = false;
    ros::Publisher potential_map_pub_;
    ros::Publisher path_dubin_pub_;
    ros::Publisher path_pub_;
    nav_msgs::OccupancyGrid potential_map_;
    /* Costmap values */
    double origin_x_costmap_, origin_y_costmap_, resolution_costmap_;
    int cells_x_costmap_, cells_y_costmap_;
    /* Parametros */
    double step_size_meters_;
    double theta_limit_;
    double wheel_base_, max_steering_angle_;
    double orientation_tolerance_, xy_tolerance_;
    int factor_cost_distance_, factor_cost_angle_;
    int factor_cost_angle_goal_;
    int factor_cost_obstacle_; 
    double divisor_factor_increment_step, distance_threshold_step;
    double penalization_distance_goal_;
    double min_step_size_, max_step_size_;
    int division_curve_, division_theta_, division_steps_; 
    double final_threshold_;
    bool enable_dubin_;
    int max_iteration_search;
  };
};

#endif