# Puma System Monitor

## Resumen

Lee y publica el estado del computador u mini computador. Realiza lecturas de uso de CPU, temperatura interna y uso de RAM.

## Dependencias

- rospy
- std_msgs

## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_system_monitor

## Ejecución

Para ejecutar se usa el roslaunch:

    roslaunch puma_system_monitor minipc_monitor.launch

Este lanzador está configurado para leer correctamente el estado del minipc implementado en el robot Puma. Para otros equipos se debe probar cambiando el parámetro `thermal_zone`, ya que varía entre equipos. Los datos los publica en:

  - **`cpu_usage`** (std_msgs/Float32)

  - **`cpu_temperature`** (std_msgs/Float32)

  - **`memory_usage`** (std_msgs/Float32)