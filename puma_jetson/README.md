# Paquetes para Jetson nano

En este directorio se encuentran los paquetes que se ejecutarán exclusivamente en la placa de desarrollo Jetson Nano.

Para la ejecución de los nodos en la Jetson puede ser necesario dar permisos de ejecución a los archivos **`.py`**. Esto se encuentra automatizado en **`make_exec.sh`**:

    source make_exec.sh
    ./make_exec.sh

Los paquetes en este directorio tienen las siguientes funciones:

- **`puma_bringup_jetson`**: Lanzador de nodos necesarios en la Jetson nano para el control del robot PUMA.

- **`puma_controller`**: Encargado del control de velocidad y dirección del robot.

- **`puma_parking`**: Control de la activación de los motores delantero y trasero del robot.

- **`puma_reverse`**: Control de la activación de la reversa tanto para el motor delantero y trasero del robot.

- **`puma_mode_control`**: Manejador del tipo de control a emplear en el robot. Distintos modos para evitar que interfiera entre sí.

- **`puma_ip_cameras`**: Maneja de las cámaras IP conectadas del robot PUMA para ser visualizadas en el entorno de ROS.
