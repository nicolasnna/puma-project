#!/bin/bash

echo "Ajustando ejecutables Python"
cd puma_robot/puma_parking/scripts || echo "No se ha encontrado puma_parking"
chmod +x puma_parking_node.py 
cd ../..
cd puma_reverse/scripts || echo "No se ha encontrado puma_reverse"
chmod +x puma_reverse_node.py 
cd ../..
cd puma_joy/scripts || echo "No se ha encontrado puma_joy"
chmod +x puma_joy_node.py 
cd ../..
cd puma_imu_driver/scripts || echo "No se ha encontrado puma_joy"
chmod +x puma_imu_driver_node.py 
cd ../..
cd puma_controller/scripts || echo "No se ha encontrado puma_joy"
chmod +x velocity_linear_node.py 
chmod +x convert_ackermann_node.py 
cd ../..
echo "Listo"