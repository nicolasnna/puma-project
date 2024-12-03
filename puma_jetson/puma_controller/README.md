# Puma Controller

## Resumen

Paquete ROS noetic para el control de velocidades del robot PUMA. En este paquete se encuentra un convertidor de velocidades lineales y angulares a ackermann (velocidad lineal y angulo de las ruedas), un control de velocidad entre velocidad lineal y valores pwm para el acelerador y por último, un nodo para publicar velocidades en el tópico **`cmd_vel`** para pruebas.

## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_controller

#### Dependencias

- [Robot Operating System (ROS)](http://wiki.ros.org)
- [geometry_msgs](http://wiki.ros.org/geometry_msgs)
- [ackermann_msgs](http://wiki.ros.org/ackermann_msgs)
- [tf2_ros](http://wiki.ros.org/tf2_ros)

## Ejecución

Los nodos pueden ejecutarse de manera individual:

    rosrun puma_controller convert_ackermann_node.py
    rosrun puma_controller puma_controller.py
    rosrun puma_controller teleop_keyboard.py

Se tiene un único launch para ejecutar tanto **`convert_ackermann_node.py`** como **`puma_controller`**, y con subida de parámetros de **`puma_controller.yaml`**:

    roslaunch puma_controller puma_velocity.launch

## Launch

- **puma_velocity.launch:** Lanzador de los nodos **`puma_controller`** y **`convert_ackermann`** para el control del robot puma a partir de velocidades lineales y angulares. Tambien se realiza la carga del archivo **`config/puma_controller.yaml`**.

## Nodos

### puma_controller

Se encarga del control del robot según los valores obtenidos en **`ackermann_msgs`**, variando el valor del acelerador, la reversa, dirección, y frenos. Adicionalmente se tiene un selector de modos para deshabilitarlo en caso de pasar a un control manual.

#### Suscriptores

- **`/puma/control/ackermann/command`** (ackermann_msgs/AckermannDriveStamped)

  Recibe los valores de control para la velocidad lineal y posición angular de las ruedas.

- **`/puma/odometry/filtered`** (nav_msgs/Odometry)

  Recibe el estado de localización actual del robot.

- **`/puma/mode_selector`** (std_msgs/String)

  Selector de modos: `autonomous` para habilitar el nodo **`puma_controller`** y cualquier otro valor para cambiar a modo manual (deshabilitar).

#### Publicadores

- **`/puma/reverse/command`** (std_msgs/Bool)

  Controla la activación de los frenos.

- **`/puma/parking/command`** (std_msgs/Bool)

  Controla la activación del motor delantero y trasero.

- **`/puma/accelerator/command`** (std_msgs/Int16)

  Controla el valor PWM del acelerador de los motores.

- **`/puma/direction/command`** (puma_direction_msgs/DirectionCmd)

  Envía el ángulo en radianes de la dirección objetivo de las ruedas delanteras.


#### Parámetros

- **`puma_controller`**

  - **`~accelerator_topic`** (String, default: "puma/accelerator/command")

    Tópico para control del acelerador.

  - **`~parking_topic`** (String, default: "puma/parking/command")

    Tópico para control de activación de motores.

  - **`~reverse_topic`** (String, default: "puma/reverse/command")

    Tópico para control de la reversa.

  - **`~ackermann_topic`** (String, default: "puma/control/ackermann/command")

    Tópico de subscripción del conversor ackermann.

  - **`~direction_topic`** (String, default: "puma/direction/command")

    Tópico para el control de dirección.

  - **`~range_accel_converter`** (Array-Int, default: [25, 35])

    Valor mínimo y máximo del acelerador a tomar en cuenta para la conversión lineal entre velocidad y pwm.

  - **`~connect_to_ackermann_converter`** (Bool, default: false)

    Habilita la conexión con nodo de conversion a ackermann para definir el angulo a usar en el robot. En caso de False, se usa la velocidad angular z de Cmd_Vel para definir el ángulo.

  - **`~kp`** (Float, default: 0.3)
  
    Componente proporcional del controlador PIDAntiWindUP para la velocidad.

  - **`~ki`** (Float, default: 0.2)
  
    Componente integral del controlador PIDAntiWindUP para la velocidad.
  
  - **`~kd`** (Float, default: 0.05)
  
    Componente derivativo del controlador PIDAntiWindUP para la velocidad.

### convert_ackermann

Se encarga de convertir velocidades lineales y angulares (tipo `Twist`) a velocidad lineal y posición angular (tipo `AckermannDriveStamped`) para luego ser usado en el nodo de control.

#### Suscriptores

- **`/cmd_vel`** (geometry_msgs/Twist)

  Recibe los comandos de velocidades objetivo.

- **`/puma/arduino/status`** (puma_arduino_msgs/StatusArduino)

  Recibe el estado de las variables en el Arduino Mega.

- **`/puma/mode_selector`** (std_msgs/String)

  Recibe los comandos para habilitar el nodo. (habilitar: "autonomous")

#### Publicadores

- **`/puma/control/ackermann/command`** (ackermann_msgs/AckermannDriveStamped)

  Comandos de velocidad transformadas para un sistema ackermann.

#### Parámetros

- **`ackermann_converter`**

  - **`~cmd_vel_topic`** (String, default: "cmd_vel")

    Tópico de comando de velocidad.

  - **`~wheel_base`** (Float, default: 1.15)

    Distancia entre el eje de las ruedas delanteras y traseras para la conversión ackermann. Valor en metros.

  - **`~ackermann_topic`** (String, default: "puma/control/ackermann/command")

    Tópico a publicar la conversión en ackermann msgs.

  - **`~arduino_status_topic`** (String, default: "puma/arduino/status")

    Tópico para revisar el estado del arduino.

### teleop_keyboard

Publica velocidad lineal y angular en el tópico **`cmd_vel`** usando las teclas `WASDX`.

#### Publicadores

- **`/cmd_vel`** (geometry_msgs/Twist)
