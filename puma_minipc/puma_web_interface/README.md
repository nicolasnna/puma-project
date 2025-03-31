# Puma Web Interface

## Resumen

Interfaz entre la aplicación Web y el robot Puma. Emplea API para cumplir funciones como la ejecución de comandos desde la web, teleoperación, actualización de logs y actualización de los sensores del robot en la nube.

## Dependencias

- rospy
- actionlib
- actionlib_msgs
- std_msgs
- puma_msgs
- requests
- base64
- json

## Instalación

Emplear catkin tools para la compilación e instalación:

    catkin build puma_web_interface

## Ejecución

Para ejecutar la interfaz web, se debe iniciar el lanzador `web_interface_node`:

    roslaunch puma_web_interface web_interface_node.py

Adicionalmente se tiene un lanzador configurado para conectarse a un servidor web local:

    roslaunch puma_web_interface web_interface_node_local.py

## Scripts

- **`get_command_backend.py`**: Obtiene comandos desde la aplicación web, los traduce en tópicos de ROS y los ejecuta en el robot.

- **`manage_teleoperation_backend.py`**: Maneja la teleoperación del robot Puma. Espera el modo `web` en el controlador y luego lee continuamente los comandos enviados desde la web para enviarlos por ROS. 

- **`send_robot_status_backend.py`**: Envia el estado del robot a la aplicación web. Envia información como imagenes de la realsense, gps, odometría, estado del Arduino, etc.

- **`update_logs_backend.py`**: Envia los logs generados por el robot a la aplicación web. 