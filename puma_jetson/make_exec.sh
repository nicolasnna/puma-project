#!/bin/bash
echo "Cambiando permisos a los nodos"

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


echo "Cambio de permisos a ejecutables terminado"
