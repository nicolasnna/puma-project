#!/bin/bash
set -e
source /home/puma/.bashrc
source /opt/ros/noetic/setup.bash
source /home/puma/puma_ws/devel/setup.bash

ROS_MASTER="10.42.0.100"
PORT="11311"

echo "Esperando a que el ROS Master ($ROS_MASTER:$PORT) esté disponible..."
while ! nc -z $ROS_MASTER $PORT; do
  sleep 5
done

echo "ROS Master detectado. Ejecutando manejador de Ip cameras con NVR.."

roslaunch puma_ip_devices publish_camera_nvr.launch