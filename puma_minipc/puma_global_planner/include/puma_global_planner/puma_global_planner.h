/**
 * Global planner for PUMA robot
 * created by: Nicolas Norambuena
 */
#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <chrono>
#include <dynamic_reconfigure/server.h>
#include <puma_global_planner/PumaGlobalPlannerConfig.h>

#ifndef PUMA_GLOBAL_PLANNER_H
#define PUMA_GLOBAL_PLANNER_H

using std::string;

struct Node
{
  double x, y, theta;
  int cell_x, cell_y;
  double x_start, x_end, y_start, y_end;
  std::shared_ptr<Node> parent;
  double distance_cost, prev_cost, total_cost;
  /* Constructor */
  Node(double x, double y, double theta, double resolution)
      : x(x), y(y), theta(theta), parent(nullptr), prev_cost(0), total_cost(0), distance_cost(0), x_start(x - resolution / 2), y_start(y - resolution / 2), x_end(x + resolution / 2), y_end(y + resolution / 2), cell_x(0), cell_y(0) {}

  /* Operador para estructuras como priority_queue*/
  bool operator<(const Node &other) const
  {
    return total_cost > other.total_cost;
  }
  /* Calcular distancia entre nodos */
  double distance(const Node &other) const
  {
    return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
  }

  bool operator==(const Node &other) const
  {
    return (cell_x == other.cell_x) && (cell_y == other.cell_y);
  }
};

/* Funcion hash */
struct NodeHash
{
  std::size_t operator()(const Node &node) const
  {
    /* Combinar x, y, theta en hash */
    std::hash<double> hash_fn;
    std::size_t hash_x = hash_fn(node.cell_x);
    std::size_t hash_y = hash_fn(node.cell_y);
    // std::size_t hash_theta = hash_fn(node.theta);

    return hash_x ^ (hash_y << 1);
  }
};

namespace puma_global_planner
{

  class PumaGlobalPlanner : public nav_core::BaseGlobalPlanner
  {
  public:
    PumaGlobalPlanner();
    PumaGlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

    /* Sobreescribir clases de la interfaz nav_core::BaseGlobalPlanner */
    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);

  private:
    /* Config */
    void loadRosParam(ros::NodeHandle &nh);
    void reconfigureCB(puma_global_planner::PumaGlobalPlannerConfig &config, uint32_t level);

    /* Busqueda del destino con A* */
    Node searchGoalAStar(const Node &start, const Node &goal);
    Node createNeighboor(const Node &current, const Node &goal, int ix, int iy);
    nav_msgs::Path transformNodeReachedToPath(const Node &node_goal_reached);
    std::vector<std::shared_ptr<Node>> getDubinsCurvePath(std::vector<std::shared_ptr<Node>> &path);

    /* Generar plan */
    nav_msgs::Path generatePlanFromNodes(const std::vector<std::shared_ptr<Node>> &plan_nodes);

    /* Comprobar si la celda del nodo abarca el destino */
    bool nodeIsGoal(const Node &node, const Node &goal);

    /* Parametros */
    double xy_goal_tolerance_;
    double resolution_;
    double turning_radius_;
    int interval_subsamples_;
    double meters_subsamples_;
    double step_size_dubins_;
    bool use_dubins_;

    /* Atributos */
    costmap_2d::Costmap2DROS *costmap_ros_;
    costmap_2d::Costmap2D *costmap_;
    ros::Publisher plan_pub_, plan_post_pub_;
    bool initialized_;
    bool setup_;
    puma_global_planner::PumaGlobalPlannerConfig default_config_;

  };
}

#endif // PUMA_GLOBAL_PLANNER_H