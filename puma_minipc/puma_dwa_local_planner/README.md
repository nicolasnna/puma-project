# Puma Hybrid Astar Planner

## Resumen

Plugin de planificador local para move_base en ROS Noetic. Este planificador esta basado en el algoritmo DWA con modificaciones, pensadas para el robot PUMA.

## Compilación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_dwa_local_planner

### Dependencias

- [costmap_2d](http://wiki.ros.org/costmap_2d)
- [geometry_msgs](http://wiki.ros.org/geometry_msgs)
- [nav_core](http://wiki.ros.org/nav_core)
- [pluginlib](http://wiki.ros.org/pluginlib)
- [nav_msgs](http://wiki.ros.org/nav_msgs)
- [tf](http://wiki.ros.org/tf)
- [roscpp](http://wiki.ros.org/roscpp)
- [visualization_msgs](http://wiki.ros.org/visualization_msgs)

## Incorporar en move_base

Para usarlo como planificador global es necesario definir el parámetro **base_local_planner**:

    <param name="base_local_planner" value="puma_dwa_local_planner/PumaDwaLocalPlanner" /> 

## puma_dwa_local_planner

Este planificador se encarga de seguir la ruta del planificador global usando el algoritmo DWA. Este planificador considera la desviacion respecto a la ruta global, angulo, obstaculo y distancia para encontrar la mejor ruta local a emplear.

Adicionalmente, en caso de no encontrar ruta valida se desplaza el robot una cierta distancia hacia atras a velocidad mínima, para volver a buscar una ruta posible.

Este planificador tiene los siguientes publicadores y suscriptores:

### Publicador

- **`cmd_vel`** (geometry_msgs/Twist)

  Publica comandos en velocidad lineal x e angular z, según la ruta óptima. La velocidad angular z entregada es el ángulo de la dirección a emplear en el robot en radianes.

- **`move_base/PumaDwaLocalPlanner/potential_trajectories`** (visualization_msgs/MarkerArray)

  Publica las trajectorias posibles para su visualización en rviz.

- **`move_base/PumaDwaLocalPlanner/local_plan`** (nav_msgs/Path)

  Publica la ruta a seguir.

### Suscriptor

- **`/<odom>`** (nav_msgs/Odometry)

  Se suscribe al topico de odometria  definido en los parámetros del planificador. Los datos recibidos se usa para el control de distancia de retroceso y velocidad a emplear.

### Parámetros

Los parámetros se ajustan en el grupo **`PumaDwaLocalPlanner`**:

- **`topic_odom`** (string, default: "odom")

  Tópico de odometria del robot.

- **`max_velocity`** (double, default: 1.0)

  Velocidad máxima a alcanzar en m/s.

- **`min_velocity`** (double, default: 1.0)

  Velocidad mínima a alcanzar en m/s. También es la velocidad a emplear para el retroceso.

- **`max_steering_angle`** (double, default: 0.52)

  Máximo ángulo de giro de la dirección del robot.

- **`acceleration_x`** (double, default: 0.2)

  Valor de la acceleración del robot.

- **`desacceleration_x`** (double, default: 0.3)

  Valor de la desacceleración del robot. Se considera al encontrarse cerca del destino.

- **`distance_for_desacceleration`** (double, default: 4.0)

  Distancia entre el robot y el destino para empezar a reducir la velocidad.

- **`reverse_limit_distance`** (double, default: 3.0)

  Distancia a retroceder en caso de no encontrar ruta válida.

- **`time_simulation`** (double, default: 2.0)

  Tiempo a simular para la creación de posibles rutas.

- **`xy_goal_tolerance`** (double, default: 0.4)

  Tolerancia en metros para la detección de llegada con el destino.

- **`steering_samples`** (int, default: 0)

  Cantidad de ángulos de dirección a evaluar para la creación de rutas. Entre -**max_steering_angle** y **max_steering_angle**.

- **`factor_cost_deviation`** (double, default: 2.0)

  Factor de costo entre la distancia final de la ruta local con la posición más cercana de la ruta global. 

- **`factor_cost_distance_goal`** (double, default: 15.0)

  Factor de costo entre la distancia final de la ruta local con la posición de destino.

- **`factor_cost_angle_to_plan`** (double, default: 5.0)

  Factor de costo entre la orientación de la posición final de la ruta local con la orientación de la posición más cercana en la ruta global.

- **`factor_cost_obstacle`** (double, default: 3.0)

  Factor de costo entre la orientación de la posición final de la ruta local con los obstáculos.