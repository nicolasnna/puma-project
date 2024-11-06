# Puma Hybrid Astar Planner

## Resumen

Plugin de planificador global para move_base en ROS Noetic. Este planificador esta basado en el algoritmo Astar Hybrid, que busca la ruta más corta tomando encuenta la busqueda por celdas considerando (x, y, theta).

## Compilación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_hybrid_astar_planner

### Dependencias

- [Robot Operating System (ROS)](http://wiki.ros.org)
- [costmap_2d](http://wiki.ros.org/costmap_2d)
- [geometry_msgs](http://wiki.ros.org/geometry_msgs)
- [nav_core](http://wiki.ros.org/nav_core)
- [pluginlib](http://wiki.ros.org/pluginlib)
- [nav_msgs](http://wiki.ros.org/nav_msgs)
- [tf](http://wiki.ros.org/tf)
- [roscpp](http://wiki.ros.org/roscpp)

## Incorporar en move_base

Para usarlo como planificador global es necesario definir el parámetro **base_global_planner**:

    <param name="base_global_planner" value="puma_hybrid_astar_planner/PumaHybridAStarPlanner"/>

## puma_hybrid_astar_planner

Este planificador se encarga de plantear la ruta a seguir para el robot. Para ello se utiliza una busqueda basado en **(x, y, theta)** considerando las limitaciones de ángulo, lo que es optimo para robots con dirección ackermann.

Para buscar la mejor ruta considera el mapa de costo, angulo respecto al destino, distancia y angulo entre curvas.

### Publicadores

- **`move_base/PumaDwaLocalPlanner/global_plan`** (nav_msgs/Path)

  Publica la ruta del plan global sin curvas dubin.

- **`move_base/PumaDwaLocalPlanner/global_dubin_plan`** (nav_msgs/Path)

  Publica la ruta del plan global con curvas dubin en puntos intermedios.

- **`move_base/PumaDwaLocalPlanner/potential_map`** (nav_msgs/OccupancyGrid)

  Publica las celdas revisadas para la busqueda de la ruta.

### Parámetros

Los parámetros se ajustan en el grupo **`PumaHybridAStarPlanner`**:

- **`step_size_meters`** (double, default: 0.3)

  Distancia entre celdas para la busqueda.

- **`division_theta`** (int, default: 4)

  Cantidad de muestras del ángulo para la busqueda de rutas. Entre -theta_limit y theta_limit.

- **`theta_limit`** (double, default: 0.628)

  Limite del angulo theta para la busqueda de rutas en radianes.

- **`xy_goal_tolerance`** (double, default: 0.4)

  Distancia de tolerancia al nodo final en metros. Dentro del rango se realiza una interpolación directa.

- **`factor_cost_distance`** (int, default: 10)

  Factor de costo asociado a la distancia entre la celda y el destino.

- **`factor_cost_angle_curve`** (int, default: 5)

  Factor de costo asociado a la curva entre nodos cercanos.

- **`factor_cost_angle_goal`** (int, default: 2)

  Factor de costo asociado a la diferencia de angulo entre la orientación posible del robot con la orientación del destino.

- **`factor_cost_obstacle`** (int, default: 10)

  Factor de costo asociado al obstaculo, si es que existe, en el mapa de costos.

- **`enable_dubin`** (bool, default: false)

  Suavizado de puntos de la ruta con curvas dubin.

- **`division_curve`** (int, default: 3)

  Cantidad de posiciones intermedias creadas al emplear suavizado con dubin.