# Puma Brake Jetson

## Resumen

Paquete ROS noetic para el control de motores paso a paso usando los GPIO de una Jetson Nano para el manejo de frenos

## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_brake_jetson

#### Dependencias

- [Robot Operating System (ROS)](http://wiki.ros.org)
- [diagnostic_msgs](http://wiki.ros.org/diagnostic_msgs)
- puma_brake_msgs
- [Jetson.GPIO](https://github.com/NVIDIA/jetson-gpio)

## Ejecución

Se puede ejecutar directamente el nodo:

    rosrun puma_brake_jetson brake_jetson_node.py

o por el archivo _launch_ para considerar el archivo de configuración:

    roslaunch puma_brake_jetson brake_jetson.launch

## Launch

- **brake_jetson.launch:** Lanzador del nodo _brake_jetson_node.py_, considerando los parámetros definidos en _config/brake_params.yaml_.

## Nodo

### brake_controller

Controla el freno delantero y trasero del Puma, usando los GPIO de la jetson y los driver A4988. Al inicio se requiere un mensaje del tipo _std_msgs/Empty_ para iniciar la calibración y luego, al control de los frenos.

#### Suscriptores

- **`/puma/brake/switch_a`** (std_msgs/Bool)

  Comprueba el estado del switch del freno delantero.

- **`/puma/brake/switch_b`** (std_msgs/Bool)

  Comprueba el estado del switch del freno trasero.

- **`/puma/brake/front_wheels/command`** (puma_brake_msgs/BrakeCmd)

  Activa o desactiva el freno delantero.

- **`/puma/brake/front_wheels/start_calibration`** (std_msgs/Empty)

  Inicia la calibración del freno delantero. (Solo se usa al iniciar el nodo).

- **`/puma/brake/rear_wheels/command`** (puma_brake_msgs/BrakeCmd)

  Activa o desactiva el freno trasero.

- **`/puma/brake/rear_wheels/start_calibration`** (std_msgs/Empty)

  Inicia la calibración del freno trasero. (Solo se usa al iniciar el nodo).

#### Publicadores

- **`/puma/brake/front_wheels/diagnostic`** (diagnostic_msgs/DiagnosticStatus)

  Publica un diagnóstico del estado actual del control del freno delantero.

- **`/puma/brake/rear_wheels/diagnostic`** (diagnostic_msgs/DiagnosticStatus)

  Publica un diagnóstico del estado actual del control del freno trasero.

#### Parámetros

- **`switch_topic_a`** (string, default: "/puma/brake/switch_a"")

  Tópico de suscripción para el switch A (freno delantero).

- **`switch_topic_b`** (string, default: "/puma/brake/switch_b")

  Tópico de suscripción para el switch B (freno trasero).

- **`topic_brake_front`** (string, default: "/puma/brake/front_wheels")

  Tópico base del freno delantero.

- **`topic_brake_rear`** (string, default: "/puma/brake/rear_wheels")

  Tópico base del freno trasero.

- **`extra_steps_front`** (int, default: 200)

  Offset pasos del motor después de detectar el switch A.

- **`extra_steps_rear`** (int, default: 200)

  Offset pasos del motor despues de detectar el switch B.

- **`pin_dir_front`** (int, default: 37)

  Pin GPIO de salida para la dirección del motor encargado del freno delantero.

- **`pin_step_front`** (int, default: 36)

  Pin GPIO de salida para los pasos del motor encargado del freno delantero.

- **`pin_dir_rear`** (int, default: 35)

  Pin GPIO de salida para la dirección del motor encargado del freno delantero.

- **`pin_step_rear`** (int, default: 33)

  Pin GPIO de salida para los pasos del motor encargado del freno delantero.
