#!/bin/bash

# Este script debe ejecutarse con privilegios de root.
if [ "$EUID" -ne 0 ]; then
  echo "Por favor, ejecuta el script como root."
  exit 1
fi

# Definir rutas de origen (asumiendo que los archivos están en el mismo directorio que el script)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROSCORE_SERVICE_SRC="${SCRIPT_DIR}/boot/puma-roscore.service"
LAUNCHUP_SERVICE_SRC="${SCRIPT_DIR}/boot/puma-minipc.service"
LAUNCHUP_SCRIPT_SRC="${SCRIPT_DIR}/boot/pumapc-bringup-ros.sh"

# Definir destinos
ROSCORE_SERVICE_DST="/etc/systemd/system/roscore.service"
LAUNCHUP_SERVICE_DST="/etc/systemd/system/pumapc-bringup-ros.service"
LAUNCHUP_SCRIPT_DST="/usr/local/bin/pumapc-bringup-ros.sh"

echo "Copiando el archivo de servicio roscore..."
if [ -f "$ROSCORE_SERVICE_SRC" ]; then
  cp -i "$ROSCORE_SERVICE_SRC" "$ROSCORE_SERVICE_DST"
  echo "Servicio copiado a ${ROSCORE_SERVICE_DST}"
else
  echo "Error: no se encontró ${ROSCORE_SERVICE_SRC}"
  exit 1
fi

echo "Copiando el archivo de servicio pumapc..."
if [ -f "$LAUNCHUP_SERVICE_SRC" ]; then
  cp -i "$LAUNCHUP_SERVICE_SRC" "$LAUNCHUP_SERVICE_DST"
  echo "Servicio copiado a ${LAUNCHUP_SERVICE_DST}"
else
  echo "Error: no se encontró ${LAUNCHUP_SERVICE_SRC}"
  exit 1
fi

echo "Copiando el script de bringup..."
if [ -f "$LAUNCHUP_SCRIPT_SRC" ]; then
  cp -i "$LAUNCHUP_SCRIPT_SRC" "$LAUNCHUP_SCRIPT_DST"
  # Asegurar que el script sea ejecutable
  chmod +x "$LAUNCHUP_SCRIPT_DST"
  echo "Script copiado y se establecieron permisos en ${LAUNCHUP_SCRIPT_DST}"
else
  echo "Error: no se encontró ${LAUNCHUP_SCRIPT_SRC}"
  exit 1
fi

echo "Recargando la configuración de systemd..."
systemctl daemon-reload

echo "Habilitando el servicio roscore..."
systemctl enable roscore.service

echo "Habilitando el servicio pumapc..."
systemctl enable pumapc-bringup-ros.service

echo "Iniciando el servicio roscore..."
systemctl start roscore.service

echo "Iniciando el servicio pumapc..."
systemctl start pumapc-bringup-ros.service

echo "Instalación completada. Los servicios roscore y pumapc se han instalado, habilitado e iniciado."