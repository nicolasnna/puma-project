#!/bin/bash

# Define la ruta absoluta del directorio donde está el script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Cambiando permisos a los nodos"

if [ -f "puma_ip_devices/scripts/reboot_ptz_node.py" ]; then
    chmod +x puma_ip_devices/scripts/reboot_ptz_node.py && echo " -> reboot_ptz_node listo !!!"
else
    echo " -X no se ha encontrado reboot_ptz_node.py"
fi

if [ -f "puma_ip_devices/scripts/publish_camera_ip.py" ]; then
    chmod +x puma_ip_devices/scripts/publish_camera_ip.py && echo " -> publish_camera_ip listo !!!"
else
    echo " -X no se ha encontrado publish_camera_ip.py"
fi

if [ -f "puma_ip_devices/scripts/speaker_manager_node.py" ]; then
    chmod +x puma_ip_devices/scripts/speaker_manager_node.py && echo " -> speaker_manager_node listo !!!"
else
    echo " -X no se ha encontrado speaker_manager_node.py"
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

if [ -f "../puma_utils/puma_system_monitor/scripts/services_manager_node.py" ]; then
    chmod +x ../puma_utils/puma_system_monitor/scripts/services_manager_node.py && echo " -> services_manager_node listo !!!"
else
    echo " -X no se ha encontrado services_manager_node.py"
fi

if [ -f "../puma_utils/puma_joy/scripts/puma_joy_node.py" ]; then
    chmod +x ../puma_utils/puma_joy/scripts/puma_joy_node.py && echo " -> puma_joy listo !!!"
else
    echo " -X no se ha encontrado puma_joy.py"
fi

if [ -f "puma_controller/scripts/puma_controller_node.py" ]; then
    chmod +x puma_controller/scripts/puma_controller_node.py && echo " -> puma_controller listo !!!"
else
    echo " -X no se ha encontrado puma_controller.py"
fi

if [ -f "puma_mode_control/scripts/control_mode_node.py" ]; then
    chmod +x puma_mode_control/scripts/control_mode_node.py && echo " -> control_mode_selector listo !!!"
else
    echo " -X no se ha encontrado control_mode_node.py"
fi

if [ -f "puma_bringup_jetson/scripts/supervisor_manager_launcher.py" ]; then
    chmod +x puma_bringup_jetson/scripts/supervisor_manager_launcher.py && echo " -> supervisor_manager_launcher listo !!!"
else
    echo " -X no se ha encontrado supervisor_manager_launcher.py"
fi

if [ -f "../puma_utils/puma_imu/scripts/translate_icm20948_arduino.py" ]; then
    chmod +x ../puma_utils/puma_imu/scripts/translate_icm20948_arduino.py && echo " -> translate_icm20948_arduino listo !!!"
else
    echo " -X no se ha encontrado translate_icm20948_arduino.py"
fi



echo "Cambio de permisos a ejecutables terminado"
