#!/bin/bash
set -e
source /home/puma/.bashrc
source /opt/ros/noetic/setup.bash
source /home/puma/puma_ws/devel/setup.bash

ROS_MASTER="10.42.0.100"
PORT="11311"

echo "Esperando a que el ROS Master ($ROS_MASTER:$PORT) esté disponible..."
while ! nc -z $ROS_MASTER $PORT; do
  sleep 2
done
echo "ROS Master disponible."

# Esperar conexión a Internet
echo "Verificando conexión a Internet..."
while ! ping -c 1 -W 2 8.8.8.8 &> /dev/null; do
  echo "No hay conexión a Internet, reintentando en 5 segundos..."
  sleep 5
done
echo "Conexión a Internet establecida."

echo "Iniciando conexión con la web..."
roslaunch puma_web_interface web_interface.launch