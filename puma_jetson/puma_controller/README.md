# Puma Controller

## Resumen

Paquete ROS noetic para el control de movimiento del robot Puma. Este control considera que el tópico `cmd_vel` entrega la velocidad linear y dirección de las ruedas. Adicionalmente puede trabajar con un tópico de entrada del tipo `ackermann_msgs/AckermannDriveStamped`. Por último, un nodo para publicar velocidades en el tópico **`cmd_vel`** para pruebas.

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

    roslaunch puma_controller puma_controller.launch

## Launch

- **puma_velocity.launch:** Lanzador de los nodos **`puma_controller`** y **`convert_ackermann`** para el control del robot puma a partir de velocidades lineales y angulares. También se realiza la carga del archivo **`config/puma_controller.yaml`**.

## Nodos

### puma_controller

Se encarga del control del robot variando el valor del acelerador, la reversa, dirección, y frenos, según el modo empleado. Para ello revisa periódicamente el modo actual del robot con `puma_mode_control`. Se tiene 4 modos válidos para emplear el control:

- `idle`: Modo en espera. Publica aceleración mínima, dirección desactivada, reversa desactivada, freno activado y deshabilita el acelerador.
- `joystick`: Modo manual con joystick. Usa los datos entregados por `puma_joy` para controlar el robot.
- `navegacion`: Modo navegación. Usa lo recibido por `cmd_vel` o en ackermann si se configura, para el control.
- `web`: Modo control por web app. Usa lo recibido desde la app web por medio de una base de datos, para el control del robot. Más información en el paquete `puma_web_interface`.

#### Suscriptores

- **`/puma/control/ackermann`** (ackermann_msgs/AckermannDriveStamped)

  Recibe los valores de control para la velocidad lineal y posición angular de las ruedas.

- **`/cmd_vel`** (geometry_msgs/Twist)

 Considera la velocidad angular Z como ángulo para el giro de ruedas. Se utiliza los valores recibidos por este tópico para el control por PID sin utilizar el convertidor ackermann.

- **`/puma/localization/ekf_odometry`** (nav_msgs/Odometry)

  Recibe el estado de localización actual del robot.

- **`/puma/control/current_mode`** (std_msgs/String)

  Obtiene el modo actual del puma, pueden ser: 'navegacion', 'web' o 'joystick'. Cualquier otro pone el robot en modo idle (aceleración 0, dirección desactivada y con frenos activados).

- **`/puma/web/teleop`** (puma_msgs/WebTeleop)

  Obtiene el comando para teleoperar el robot por medio de la web cuando está en modo 'web'.

- **`/puma/arduino/status`** (puma_msgs/StatusArduino)

  Recibe el estado del arduino Mega, comprueba si se ha activado la señal de seguridad.

- **`/move_base/status`** (actionlib_msgs/GoalStatusArray)

  Revisa el estado de `move_base`, en caso de no detectar su funcionamiento en un tiempo determinado, deshabilita el control en el modo `navegacion`.

#### Publicadores

- **`/puma/control/reverse`** (std_msgs/Bool)

  Controla la activación de los frenos.

- **`/puma/control/parking`** (std_msgs/Bool)

  Controla la activación del motor delantero y trasero.

- **`/puma/control/brake`** (std_msgs/Bool)

  Controla los frenos.

- **`/puma/control/accelerator`** (std_msgs/Int16)

  Controla el valor PWM del acelerador de los motores.

- **`/puma/control/direction`** (puma_msgs/DirectionCmd)

  Envía el ángulo en radianes de la dirección objetivo de las ruedas delanteras.

- **`/puma/logs/add_log`** (puma_msgs/Log)

  Envía logs respecto al control.

#### Parámetros

- **`puma_controller`**

  - **`~range_accel_converter`** (Array-Int, default: [25, 35])

    Valor mínimo y máximo del acelerador a tomar en cuenta para la conversión lineal entre velocidad y pwm.

  - **`~limit_angle_degree`** (int, default: 45)

    Valor límite para el ángulo de la dirección, tanto mínimo como máximo.

  - **`~connect_to_ackermann_converter`** (Bool, default: false)

    Habilita la conexión con nodo de conversion a ackermann para definir el angulo a usar en el robot. En caso de False, se usa la velocidad angular Z de Cmd_Vel para definir el ángulo.

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

  Recibe los comandos de velocidades objetivos.

- **`/puma/arduino/status`** (puma_arduino_msgs/StatusArduino)

  Recibe el estado de las variables en el Arduino Mega.

- **`/puma/control/current_mode`** (std_msgs/String)

  Recibe los comandos para habilitar el nodo. (habilitar: "autonomous")

#### Publicadores

- **`/puma/control/ackermann`** (ackermann_msgs/AckermannDriveStamped)

  Comandos de velocidad transformada para un sistema ackermann.

#### Parámetros

- **`ackermann_converter`**

  - **`~cmd_vel_topic`** (String, default: "cmd_vel")

    Tópico de comando de velocidad.

  - **`~wheel_base`** (Float, default: 1.15)

    Distancia entre el eje de las ruedas delanteras y traseras para la conversión ackermann. Valor en metros.


### teleop_keyboard

Publica velocidad lineal y angular en el tópico **`cmd_vel`** usando las teclas `WASDX`.

#### Publicadores

- **`/cmd_vel`** (geometry_msgs/Twist)
