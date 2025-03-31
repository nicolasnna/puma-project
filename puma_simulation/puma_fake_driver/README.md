# Puma Fake Driver

## Resumen

Paquete para ROS Noetic para simular el comportamiento del robot PUMA.

## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_fake_driver

### Dependencias

- [rospy](http://wiki.ros.org/rospy)
- [controller_manager](http://wiki.ros.org/controller_manager)
- [diagnostic_msgs](http://wiki.ros.org/move_base_msgs)
- [std_msgs](http://wiki.ros.org/std_msgs)
- puma_msgs

## Scripts

El paquete contiene 4 controladores:

- **`arduino_controller`**: Simula la publicación del estado del Arduino Mega.
- **`direction_controller`**: Simula el comportamiento del control de dirección del robot.
- **`tachometer_controller`**: Simula el comportamiento del tacómetro para el cálculo de la odometría.
- **`wheel_controller`**: Simula el comportamiento del acelerador según la PWM entregada. Además de considerar la reversa.
