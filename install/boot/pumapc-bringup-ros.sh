#!/bin/bash
set -e
source /home/puma/.bashrc
source /opt/ros/noetic/setup.bash
source /home/puma/puma_ws/devel/setup.bash

if [ -e /dev/ttyUSB0 ]; then
  sudo chmod a+rw /dev/ttyUSB0
fi

ROS_MASTER="10.42.0.100"
PORT="11311"

echo "Esperando a que el ROS Master ($ROS_MASTER:$PORT) esté disponible..."
while ! nc -z $ROS_MASTER $PORT; do
  sleep 2
done

roslaunch puma_bringup puma_bringup.launch