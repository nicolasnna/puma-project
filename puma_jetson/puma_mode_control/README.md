# Puma Mode Control

## Resumen 

Encargado de establecer el modo de control que se está ejecutando en el robot. Sin él, no se emplea el control realizado por `puma_controller`.

## Dependencias

- rospy
- std_msgs

## Instalación 

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_mode_control

## Ejecución

Contiene un único script:

    rosrun puma_mode_control control_mode_node.py

Por defecto publica el estado `idle`, en el que establece que el robot está en modo espera. Para la publicación y cambio de modo usa los siguientes tópicos:

- **`puma/control/change_mode`** (std_msgs/String): Realiza el cambio de modo hacia el indicado por el Tópico.
- **`puma/control/current_mode`** (std_msgs/String): Realiza la publicación de forma periódica del modo de control en que se encuentra el robot.
