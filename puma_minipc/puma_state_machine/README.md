# Puma puma_state_machine

## Resumen

Paquete ROS noetic el control de navegación del robot PUMA mediante waypoints. Para ello se utiliza una maquina de estados con [smach](http://wiki.ros.org/smach).

## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_state_machine

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

    roslaunch puma_state_machine state_machine_v2.launch

## Launch

- **state_machine_v2.launch:** Lanzador del control de waypoints con los parámetros indicados en **`config/state_machine_params.yaml`**.

## Parámetros

  - **`~add_pose_topic`** (String, default: "/initialpose")

    Tópico a suscribirse para recibir las posiciones en 2D.

  - **`~ns_topic`** (String, default: "")

    Prefijo para todos los tópicos publicadores y suscriptores internos del nodo.

  - **`~gps_topic`** (String, default: "/puma/sensors/gps/fix")

    Tópico de los datos del GPS del robot.

  - **`~odometry_topic`** (String, default: "/puma/localization/ekf_odometry")

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

    Máxima velocidad lineal a alcanzar por el robot en el modo carga. En m/s.

## puma_state_machine

Se encarga del control de la navegación del robot mediante waypoints. Este nodo funciona mediante una máquina de estados, donde en cada uno se tiene funciones e interacciones diferentes, que se detallan a continuación:

### 1. PLAN_CONFIGURATION

Estado inicial del robot, se encarga de recibir los waypoints donde tiene que llegar el robot durante la navegación. Puede recibir estas ubicaciones mediante `nav 2d goal` de rviz o en un arreglo de puntos GPS.

De este estado se puede transicionar al estado **`RUN_PLAN`** o **`RUN_PARKING_STATION_CHARGE`**.

#### Publicador

- **`<ns_topic>/path_planned`** (geometry_msgs/PoseArray)

  Indica las posiciones de destino de la navegación.

- **`/puma/navigation/files_manager`** (puma_nav_manager/ImportExportPlanAction)

  Conecta con el servidor de acción **`files_manager`** para la carga y guardado de los planes.

- **`/puma/navigation/waypoints_manager`** (puma_nav_manager/WaypointsManagerAction)

  Conecta con el servidor de acción **`waypoints_manager`** para la carga y guardado de los waypoints.

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

- **`/puma/localization/ekf_odometry`** (nav_msgs/Odometry)

  Obtiene posición local actual para el cálculo de destinos mediante GPS.

- **`/initialpose`** (geometry_msgs/PoseWithCovarianceStamped)

  Obtiene una posición de destino en 2D mediante la función de rviz.

- **`<ns_topic>/plan_configuration`** (puma_state_machine/PlanConfiguration)

  Server action para la configuración del plan de navegación. Igual al uso de tópicos pero con feedback.

### 2. RUN_PLAN

Estado del robot en el que se realiza la navegación en los waypoints anteriormente indicados.

Si se realiza tanto con exito o con fallos, vuelve al estado **`PLAN_CONFIGURATION`**.

#### Publicador

- **`/puma/navigation/waypoints_manager`** (puma_nav_manager/WaypointsManagerAction)

  Conecta con el servidor de acción **`waypoints_manager`** para la carga y guardado de los waypoints.

- **`/puma/statistics`** (puma_robot_status/RobotStatisticsAction)

  Conecta con el action server **`robot_statistics`** para la grabación de datos del robot.

- **`/puma/control/change_mode`** (std_msgs/String)

  Comando para cambiar el modo de control del robot.

#### Subscriptor

- **`<ns_topic>/plan_stop`** (std_msgs/Empty)

  Comando para detener la navegación, volviendo al estado **`PLAN_CONFIGURATION`**.

- **`/puma/arduino/status`** (puma_msgs/StatusArduino)

  Obtiene el estado del arduino Mega, para revisar si fue activado la señal de parada de emergencia.

- **`/move_base/status`** (actionlib_msgs/GoalStatusArray)

  Revisa si se sigue ejecutando el stack de navegación.

### 3. RUN_PLAN_CUSTOM

Igual que el estado `RUN_PLAN`, pero emplea más ajustes para la navegación entre waypoints como por ejemplo la repetición, tiempo entre recorridos, etc.

### 4. RUN_PARKING_STATION_CHARGE

Estado del robot para desplazarce hasta la base de carga y quedarse estacionado. Para el desplazamiento es necesario que el apriltag esté visible para la cámara del robot. Una vez finalizado, cambia al estado **`PLAN_CONFIGURATION`**.

#### Publicador

- **`/puma/tag_detector/enable`** (std_msgs/Bool)

  Tópico para activar el detector de tags.

- **`/cmd_vel`** (geometry_msgs/Twist)

  Tópico para comandar la velocidad del robot.

- **`/move_base/cancel`** (actionlib_msgs/GoalID)

  Comando para cancelar la navegación con **`move_base`**.

- **`/puma/control/change_mode`** (std_msgs/String)

  Comando para cambiar el modo de control del robot.

#### Suscriptor

- **`/puma/tag_detector/pose2camera/tag_<number>`** (geometry_msgs/PoseStamped)

  Tópico para la obtención de la posición relativa del tag.

- **`<ns_topic>/plan_stop`** (std_msgs/Empty)

  Comando para detener la navegación, volviendo al estado **`GET_PATH`**.

- **`/tag_detections`** (apriltag_ros/AprilTagDetectionArray)

  Tópico que indica que tags han sido localizados por **`apriltag_ros`**.

- **`/puma/tag_detector/goal/tag_<number>`** (move_base_msgs/MoveBaseGoal)

  Posición 2D de destino referente al tag.


