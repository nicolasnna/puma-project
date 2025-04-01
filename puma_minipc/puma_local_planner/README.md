# Puma Local Planner

## Resumen

Plugin de planificador local para `move_base` en ROS Noetic. Este planificador está basado en el algoritmo DWA con modificaciones, pensadas para el robot PUMA.

## Compilación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

```bash
    catkin build puma_local_planner
```

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

Para usarlo como planificador local es necesario definir el parámetro **base_local_planner**:

```xml
    <param name="base_local_planner" value="puma_local_planner/PumaLocalPlanner" /> 
```

## puma_local_planner

Este planificador se encarga de seguir la ruta del planificador global usando el algoritmo DWA. Este planificador se centra en seguir con mayor semejanza posible, la ruta planteada. Además considera un factor para darle una cierta prioridad a la velocidad utilizada, lo que permite aumentar la velocidad de desplazamiento del robot, sin perder la ruta planteada.

Este planificador tiene los siguientes publicadores y suscriptores:

### Publicador

- **`cmd_vel`** (geometry_msgs/Twist)

  Publica comandos en velocidad lineal X y angular z, según la ruta óptima. La velocidad angular Z entregada es el ángulo de la dirección a emplear en el robot en radianes.

- **`move_base/PumaLocalPlanner/potential_trajectories`** (visualization_msgs/MarkerArray)
  
  Publica las trajectorias posibles para su visualización en rviz.

- **`move_base/PumaLocalPlanner/local_plan`** (nav_msgs/Path)
  
  Publica la ruta a seguir.

- **`move_base/PumaLocalPlanner/global_plan`** (nav_msgs/Path)
  
  Publica la ruta global restante.

### Subscriptor

- **`<odom>`** (nav_msgs/Odometry)

  Se suscribe al topico de odometria definido en los parámetros del planificador. Los datos recibidos se usa para el control de distancia de retroceso y velocidad a emplear.

### Parámetros

- **`topic_odom`** (string, default: "odom")

  Tópico de odometria del robot.

- **`max_index_path_compare`** (int, default: 10)

  Cantidad de puntos máximos a considerar en la comparación entre la trayectoria global y la trayectoria local.

- **`max_velocity`** (double, default: 1.0)

  Velocidad máxima a alcanzar en m/s.

- **`min_velocity`** (double, default: 0.1)

  Velocidad mínima a alcanzar en m/s.

- **`max_acceleration`** (double, default: 0.5)
  
  Aceleración máxima a alcanzar en m/s^2.

- **`max_deceleration`** (double, default: 0.5)
  
  Desaceleración máxima a alcanzar en m/s^2. Usando cuando se llega al destino

- **`distance_for_deceleration`** (double, default: 1.0)

  Distancia mínima al destino para comenzar a desacelerar.

- **`steering_rads_limit`** (double, default: 0.698)

  Máximo ángulo de giro de la dirección del robot en radianes.

- **`distance_reverse`** (double, default: 0.2)

  Distancia a retroceder en caso de no encontrar ruta válida.

- **`time_simulation`** (double, default: 1.0)

  Tiempo a simular para la creación de posibles rutas.

- **`time_step`** (double, default: 0.1)

  Tiempo entre cada paso de simulación.

- **`time_simulation_reverse`** (double, default: 2.0)

  Tiempo a simular para la creación de posibles rutas en retroceso.

- **`xy_goal_tolerance`** (double, default: 0.5)

  Tolerancia en metros para la detección de llegada con el destino.

- **`steering_samples`** (int, default: 10) 

  Cantidad de ángulos de dirección a evaluar para la creación de rutas. Entre -**steering_rads_limit** y **steering_rads_limit**.

- **`velocity_samples`** (int, default: 1)

  Cantidad de velocidades a evaluar para la creación de rutas. Entre **min_velocity** y **max_velocity**.

- **`factor_velocity`** (double, default: 1.0)

  Factor de costo de la velocidad elegida para la navegación, a valor más alto, más prioridad le da a la velocidad, por sobre a la semejanza con la ruta global.
