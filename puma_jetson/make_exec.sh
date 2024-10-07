#!/bin/bash
echo "Cambiando permisos a los nodos"

if [ -f "puma_brake_jetson/scripts/brake_jetson_node.py" ]; then
    chmod +x puma_brake_jetson/scripts/brake_jetson_node.py && echo " -> brake_jetson_node listo !!!"
else
    echo " -X no se ha encontrado brake_jetson_node.py"
fi

if [ -f "puma_imu_driver/scripts/puma_imu_driver_node.py" ]; then
    chmod +x puma_imu_driver/scripts/puma_imu_driver_node.py && echo " -> puma_imu_driver_node listo !!!"
else
    echo " -X no se ha encontrado puma_imu_driver_node.py"
fi

if [ -f "puma_parking/scripts/puma_parking_node.py" ]; then
    chmod +x puma_parking/scripts/puma_parking_node.py && echo " -> puma_parking_node listo !!!"
else
    echo " -X no se ha encontrado puma_parking_node.py"
fi

if [ -f "puma_reverse/scripts/puma_reverse_node.py" ]; then
    chmod +x puma_reverse/scripts/puma_reverse_node.py && echo " -> puma_reverse_node listo !!!"
else
    echo " -X no se ha encontrado puma_reverse_node.py"
fi

if [ -f "../puma_utils/puma_system_monitor/scripts/system_monitor_node.py" ]; then
    chmod +x ../puma_utils/puma_system_monitor/scripts/system_monitor_node.py && echo " -> system_monitor_node listo !!!"
else
    echo " -X no se ha encontrado system_monitor_node.py"
fi

echo "Cambio de permisos a ejecutables terminado"
