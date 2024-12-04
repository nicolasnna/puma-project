# Puma Reverse

## Resumen

Paquete ROS noetic para la activación y desactivación de la reversa de los motores del robot PUMA con la Jetson Nano.

## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_reverse

#### Dependencias

- [Robot Operating System (ROS)](http://wiki.ros.org)
- [std_msgs](http://wiki.ros.org/std_msgs)
- [Jetson.GPIO](https://github.com/NVIDIA/jetson-gpio)

## Ejecución

Se puede ejecutar directamente el nodo:

    rosrun puma_reversa puma_reversa_node.py

o por el archivo _launch_ para considerar el archivo de configuración:

    roslaunch puma_reversa puma_reversa.launch

## Nodo

### puma_reverse

Controla la activación de la reversa los motores del robot al accionar el transistor en la placa incluida en la Jetson. Esta configurada la compuerta del transistor con el pin 13 y conecta los 5V de la conexión del controlador de los motores con el GND. (Se requiere conexión del GND del robot con la placa extra en la Jetson)

#### Suscriptores

- **`/puma/control/reverse`** (std_msgs/Bool)

  Recibe el comando para activar o desactivar la reversa de los motores. Por defecto esta desactivado.

#### Parámetros

- **`reverse_pin`** (int, default: 13)

  Pin GPIO de la Jetson nano para la activación de la compuerta del transistor.
