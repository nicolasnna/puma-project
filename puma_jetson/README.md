# Paquetes para Jetson nano

En este directorio se encuentran los paquetes que se ejecutarán exclusivamente en la placa de desarrollo Jetson Nano.

Para la ejecución de los nodos en la Jetson puede ser necesario dar permisos de ejecución a los archivos **`.py`**. Esto se encuentra automatizado en **`make_exec.sh`**:

    source make_exec.sh

Los paquetes en este directorio tienen las siguientes funciones:

- **`puma_bringup_jetson`**: Lanzador de nodos necesarios en la Jetson nano para el control del robot PUMA.

- **`puma_parking`**: Control de la activación de los motores delantero y trasero del robot.

- **`puma_reverse`**: Control de la activación de la reversa tanto para el motor delantero y trasero del robot.

- **`puma_brake_jetson`**: Control de motores paso a paso para los frenos con los gpio de los frenos. (En pausa, se da prioridad a control a través de arduino)

- **`puma_imu_driver`**: Conexión con IMU bno08x con la Jetson mediante I2C. (En pausa, debido a falla de la IMU)
