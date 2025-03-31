# Paquetes para Mini PC

En este directorio se encuentran los paquetes que se ejecutarán exclusivamente para el PC central del robot

Los paquetes en este directorio tienen las siguientes funciones:

- **`puma_bringup`**: Lanzador de nodos necesarios en la PC central para el control del robot PUMA.

- **`puma_dwa_local_planner`**: Planificador local como plugin de `move_base` con el enfoque DWA para el robot PUMA. Está integrado para el uso de Waypoints.

- **`puma_hybrid_astar_planner`**: Planificador global como plugin de `move_base` con el enfoque Hybrid Astar, busqueda a partir de heuristica y considerando angulo entre puntos. Está integrado para el uso de Waypoints.

- **`puma_local_planner`**: Planificador local para Puma, utiliza enfoque DWA con mejor optimización que `puma_dwa_local_planner`.

- **`puma_global_planner`**: Planificador global para puma. Emplea busqueda por celdas usando A Star, luego se implementa un postprocesado de la ruta con curvas dubins para optimizar el plan para el robot.

- **`puma_model_3d`**: Modelo del robot Puma en urdf xacro, incluyendo plugin de gazebo para la simulación de sensores.

- **`puma_model_dualcam`**: Modelo del robot Puma al igual que `puma_model_3d` pero incluyendo una cámara realsense en la parte posterior del robot.

- **`puma_nav_manager`**: Manejador de la navegación, define los waypoints y sus estados. Incluye importación y exportación. También se incluye una interfaz para la configuración de localización por `robot_localization`, sirve para definir la orientación del robot o para limpiar la posición.

- **`puma_odometry`**: Calcula la odometría del robot a patir de las ruedas, con las mediciones del motor.

- **`puma_plugin_rviz`**: Crea plugin especificos en rviz, para facilitar el uso del sistema del paquete **`puma_state_machine`**.

- **`puma_robot_status`**: Publica y maneja la información del estado del robot como el estado de la batería, loggers de eventos, etc. Incluye un server action para la grabación de sensores del robot, para almacenarlos en local o en la nube.

- **`puma_state_machine`**: Control autónomo del robot empleando una máquina de estados. Se puede definir plan a seguir, modo de estacionamiento con la estación de carga, etc.

- **`puma_tag_detector`**: Transforma la detección del apriltag a una posición de destino.

- **`puma_web_interface`**: Interfaz para la conexión con la API REST de la nube. Entre las funciones que cumple está la traducción de comandos teleop desde la web, ejecución de comandos desde la web, publicación de logs, publicación de datos del robot en la nube, etc.