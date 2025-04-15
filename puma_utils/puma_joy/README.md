# Puma Joy

## Resumen

Interfaz para el control del robot Puma usando un mando de Xbox360, usando ROS Noetic.

## Dependencias

- joy
- sensor_msgs
- puma_msgs
- std_msgs
- rospy

## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_joy

## Ejecución 

Se usa el lanzador roslaunch **`puma_joy.launch`** para la ejecución:

    roslaunch puma_joy puma_joy.launch

Ejecuta tanto el paquete **`joy`** encargado de la lectura del control y publicación en ROS, como de la interfaz encargada de traducir el estado de los botones a tópicos específicos para el control del robot.

En **`param/joy_params.yaml`** se configura ciertos parámetros del control como aceleración mínima y máxima del robot. Son valores de la PWM entre 0 y 255. También se puede ajustar el valor de la dirección. 


> [!WARNING]
> No es recomendable usar valores altos en la PWM, el robot PUMA posee mucha potencia y por ende, si se está en un espacio limitado, será difícil de controlar. 

> [!NOTE]
> Se debe tener cuidado en el ángulo de la dirección para no ocasionar daños en el robot, igualmente, el Arduino Mega emplea un filtro de máximos y mínimos para mantener la dirección en cierto margen.

Los controles configurados son:

- **`start`**: Para iniciar el uso de control. Es necesario cada vez que se conecte el mando.
- **`RT`**: Controla el acelerador del robot.
- **`LT`**: Activa o desactiva el freno.
- **`Analogo izquierdo + A`**: Mueve la dirección del robot.
- **`LB`**: Mantenerlo pulsado activa el modo reversa.
- **`RB`**: Mantenerlo pulsado desactiva el control del motor, por ende, desactiva la aceleración.

Cuenta con servidor _reconfigure_ para ajustar parámtros como acelerador y ángulo de dirección mientras se encuentra en ejecución el programa.

