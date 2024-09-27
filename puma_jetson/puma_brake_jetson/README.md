# Puma Brake Jetson

## Resumen

Paquete ROS noetic para el control de motores paso a paso usando los GPIO de una Jetson Nano para el manejo de frenos

## Instalación

Para la instalación se realiza por catkin build (no se ha testeado con catkin make)

    catkin build puma_brake_jetson

#### Dependencias

- [Robot Operating System (ROS)](http://wiki.ros.org)
- [diagnostic_msgs](http://wiki.ros.org/diagnostic_msgs)
- puma_brake_msgs

## Usage

Describe the quickest way to run this software, for example:

Run the main node with

    roslaunch ros_package_template ros_package_template.launch

## Config files

Config file folder/set 1

- **config_file_1.yaml** Shortly explain the content of this config file

Config file folder/set 2

- **...**

## Launch files

- **launch_file_1.launch:** shortly explain what is launched (e.g standard simulation, simulation with gdb,...)

  Argument set 1

  - **`argument_1`** Short description (e.g. as commented in launch file). Default: `default_value`.

  Argument set 2

  - **`...`**

- **...**

## Nodes

### ros_package_template

Reads temperature measurements and computed the average.

#### Subscribed Topics

- **`/temperature`** ([sensor_msgs/Temperature])

  The temperature measurements from which the average is computed.

#### Published Topics

...

#### Services

- **`get_average`** ([std_srvs/Trigger])

  Returns information about the current average. For example, you can trigger the computation from the console with

      rosservice call /ros_package_template/get_average

#### Parameters

- **`subscriber_topic`** (string, default: "/temperature")

  The name of the input topic.

- **`cache_size`** (int, default: 200, min: 0, max: 1000)

  The size of the cache.

### NODE_B_NAME

...
