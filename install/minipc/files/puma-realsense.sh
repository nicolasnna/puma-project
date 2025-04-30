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

echo "ROS Master disponible. Iniciando la camara realsense ROS en 30 segundos..."

sleep 30
roslaunch puma_bringup realsense_camera.launch camera_name:=camera_front serial_no:=215122254316
