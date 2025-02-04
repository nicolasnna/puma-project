#!/bin/bash
set -e  # Detiene el script si hay errores
source /home/puma/.bashrc
source /opt/ros/melodic/setup.bash
source /home/puma/puma_ws/devel/setup.bash

sudo chown root.gpio /dev/gpiochip0
sudo chmod 660 /dev/gpiochip0
sudo chown root.gpio /dev/gpiochip1
sudo chmod 660 /dev/gpiochip1
sudo usermod -a -G dialout puma
sudo chmod a+rw /dev/ttyUSB0

ROS_MASTER="10.42.0.199"
PORT="11311"

echo "Esperando a que el ROS Master ($ROS_MASTER:$PORT) esté disponible..."
while ! nc -z $ROS_MASTER $PORT; do
  sleep 2
done

roslaunch puma_bringup_jetson puma_bringup.launch

