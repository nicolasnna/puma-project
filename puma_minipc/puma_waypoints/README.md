# Puma Waypoints

## Resumen

Paquete ROS noetic el control de navegación del robot PUMA mediante waypoints. Para ello se utiliza una maquina de estados con [smach](http://wiki.ros.org/smach).

## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_waypoints

### Dependencias

- [Robot Operating System (ROS)](http://wiki.ros.org)
- [smach_ros](http://wiki.ros.org/smach)
- [geometry_msgs](http://wiki.ros.org/geometry_msgs)
- [std_msgs](http://wiki.ros.org/std_msgs)
- puma_waypoints_msgs
- [nav_msgs](http://wiki.ros.org/nav_msgs)
- [sensor_msgs](http://wiki.ros.org/sensor_msgs)
- [tf](http://wiki.ros.org/tf)
- [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure)

## Ejecución

El nodo se ejecuta mediante _roslaunch_:

    roslaunch puma_waypoints waypoints.launch

## Launch

- **waypoints.launch:** Lanzador del control de waypoints con los parámetros indicados en **`config/waypoints_params.yaml`**.

## Parámetros

  - **`~add_pose_topic`** (String, default: "/initialpose")

    Tópico a suscribirse para recibir las posiciones en 2D.

  - **`~ns_topic`** (String, default: "")

    Prefijo para todos los tópicos publicadores y suscriptores internos del nodo.

  - **`~gps_topic`** (String, default: "/puma/sensors/gps/fix")

    Tópico de los datos del GPS del robot.

  - **`~odometry_topic`** (String, default: "/puma/odometry/filtered")

    Tópico de los datos de la odometría del robot.

  - **`~change_mode_topic`** (String, default: "/puma/control/change_mode")

    Tópico de para el cambio de la configuración del controllador del robot puma.

  - **`~goal_frame_id`** (String, default: "map")

    Marco de coordenadas de referencia para las posiciones de destino la navegación.

  - **`~odom_frame_id`** (String, default: "odom")

    Marco de coordenadas de referencia para la odometría.

  - **`~base_frame_id`** (String, default: "base_link")

    Marco de coordenadas de referencia para la base del robot.

  - **`~distance_tolerance`** (Float, default: 1.2)

    Distancia mínima para cambiar al siguiente destino objetivo.

  - **`~wait_duration`** (Float, default: 4.0)

    Tiempo de espera máximo para la espera de las tf.

  - **`~enable_topic`** (String, default: "/puma/tag_detector/enable")

    Tópico para habilitar el **`tag_detector`**.

  - **`~dis2tag_goal`** (Float, default: 1.0)

    Distancia entre la cámara del robot y el tag a alcanzar en metros.

  - **`~pid`**

    Parámetros del controlador PID.

    - **`~kp`** (Float, default: 0.1)

    - **`~ki`** (Float, default: 0.01)

    - **`~kd`** (Float, default: 0.05)

  - **`~max_vel`** (Float, default: 0.4)

    Máxima velocidad lineal a alcanzar por el robot. En m/s.

## puma_waypoints

Se encarga del control de la navegación del robot mediante waypoints. Este nodo funciona mediante una maquina de estados, donde en cada uno se tiene funcionese interacciones diferentes, que se detallan a continuación:

### 1. GET_PATH

Estado inicial del robot, se encarga de recibir los waypoints donde tiene que llegar el robot durante la navegación. Puede recibir estas ubicaciones mediante `nav 2d goal` de rviz o en un arreglo de puntos GPS.

De este estado se puede transicionar al estado **`FOLLOW_PATH`** o **`CHARGE_MODE`**.

#### Publicador

- **`<ns_topic>/path_planned`** (geometry_msgs/PoseArray)

  Indica las posiciones de destino de la navegación.

- **`<ns_topic>/path_completed`** (geometry_msgs/PoseArray)

  Indica las posiciones de destino compeltadas.

#### Suscriptor

- **`<ns_topic>/plan_reset`** (std_msgs/Empty)

  Comando para borrar el plan actual.

- **`<ns_topic>plan_upload`** (std_msgs/String)

  Recibe el nombre del archivo con las posiciones de destino a cargar.

- **`<ns_topic>/plan_save`** (std_msgs/String)

  Recibe un nombre de archivo en el que se guardará el plan actual.

- **`<ns_topic>/plan_ready`** (std_msgs/Empty)

  Comando para iniciar el plan actual. (Se necesita al menos un destino asignado) Con este comando se cambia al estado **`PATH_FOLLOW`**.

- **`<ns_topic>/run_charge_mode`** (std_msgs/Empty)

  Comando para cambiar al estado **`CHARGE_MODE`**.

- **`<ns_topic>/planned_goal_gps`** (puma_waypoints_msgs/GoalGpsArray)

  Obtiene un arreglo de posiciones de destino en formato latitud, longitud y altitud.

- **`/puma/sensors/gps/fix`** (nav_msgs/NavSatFix)

  Obtiene ubicación GPS actual para el cálculo de destinos mediante GPS.

- **`/puma/odometry/filtered`** (nav_msgs/Odometry)

  Obtiene posición local actual para el cálculo de destinos mediante GPS.

- **`/initialpose`** (geometry_msgs/PoseWithCovarianceStamped)

  Obtiene una posición de destino en 2D mediante la función de rviz.


### 2. FOLLOW_PATH

Estado del robot en el que se realiza la navegación en los waypoints anteriormente indicados.

Si se completa correctamente, se pasa al estado **`COMPLETE_PATH`** y en caso de abortar se pasa al estado **`GET_PATH`**.

#### Publicador

- **`<ns_topic>/path_planned`** (geometry_msgs/PoseArray)

  Publica un arreglo de posiciones de los destinos planeados.

- **`<ns_topic>/path_completed`** (geometry_msgs/PoseArray)

  Publica un arreglo de posiciones de los destinos completados.

- **`/puma/mode_selector`** (std_msgs/String)

  Publica el comando para habilitar el nodo **`puma_controller`**.


#### Subscriptor

- **`<ns_topic>/plan_stop`** (std_msgs/Empty)

  Comando para detener la navegación, volviendo al estado **`GET_PATH`**.


### 3. COMPLETE_PATH

Estado del robot ejecutado cuando se completa la navegación del estado **`FOLLOW_PATH`**. Desde este estado, se puede pasar a **`GET_PATH`**, **`FOLLOW_PATH`** y **`CHARGE_MODE`**.

#### Suscriptor

- **`<ns_topic>/plan_ready`** (std_msgs/Empty)

  Comando para volver a ejecutar la navegación.

- **`<ns_topic>/plan_reset`** (std_msgs/Empty)

  Comando para volver al estado de selección de rutas.

- **`<ns_topic>/run_charge_mode`** (std_msgs/Empty)

  Comando para cambiar al estado **`CHARGE_MODE`**.

### 4. CHARGE_MODE

Estado del robot para desplazarce hasta la base de carga y quedarse estacionado. Para el desplazamiento es necesario que el apriltag este visible para la cámara del robot. Una vez finalizado, cambia al estado **`GET_PATH`**.

#### Publicador

- **`/puma/tag_detector/enable`** (std_msgs/Bool)

  Tópico para activar el detector de tags.

- **`/cmd_vel`** (geometry_msgs/Twist)

  Tópico para comandar la velocidad del robot.

- **`/move_base/cancel`** (actionlib_msgs/GoalID)

  Comando para cancelar la navegación con **`move_base`**.

#### Suscriptor

- **`/puma/tag_detector/pose2camera/tag_<number>`** (geometry_msgs/PoseStamped)

  Tópico para la obtención de la posición relativa del tag.

- **`<ns_topic>/plan_stop`** (std_msgs/Empty)

  Comando para detener la navegación, volviendo al estado **`GET_PATH`**.

- **`/tag_detections`** (apriltag_ros/AprilTagDetectionArray)

  Tópico que indica que tags han sido localizados por **`apriltag_ros`**.

- **`/puma/tag_detector/goal/tag_<number>`** (move_base_msgs/MoveBaseGoal)

  Posición 2D de destino referente al tag.


