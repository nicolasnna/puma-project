# Paquetes para Mini PC

En este directorio se encuentran los paquetes que se ejecutarán exclusivamente para el PC central del robot

Los paquetes en este directorio tienen las siguientes funciones:

- **`puma_bringup`**: Lanzador de nodos necesarios en la PC central para el control del robot PUMA.

- **`puma_controller`**: Encargado del control de velocidad y dirección del robot.

- **`puma_odometry`**: Calcula la odometría del robot a patir de las ruedas, con las mediciones del motor.

- **`puma_plugin_rviz`**: Crea plugin especificos en rviz, para facilitar el uso del sistema del paquete **`puma_waypoints`**.

- **`puma_tag_detector`**: Transforma la detección del apriltag a una posición de destino.
- **`puma_waypoints`**: Control de navegación y comandos del robot mediante waypoints.
