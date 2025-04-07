#!/bin/bash

if [ "$EUID" -ne 0 ]; then
  echo "Por favor, ejecuta el script como root."
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCHUP_SCRIPT_SRC="${SCRIPT_DIR}/boot/minipc-monitor-arduino.sh"
LAUNCHUP_SERVICE_SRC="${SCRIPT_DIR}/boot/minipc-monitor-arduino.service"

# Definir destinos
LAUNCHUP_SCRIPT_DST="/usr/local/bin/minipc-monitor-arduino.sh"
LAUNCHUP_SERVICE_DST="/etc/systemd/system/minipc-monitor-arduino.service"

echo "Copiando el script de minipc monitor & arduino..."
if [ -f "$LAUNCHUP_SCRIPT_SRC" ]; then
  cp -i "$LAUNCHUP_SCRIPT_SRC" "$LAUNCHUP_SCRIPT_DST"
  # Asegurar que el script sea ejecutable
  chmod +x "$LAUNCHUP_SCRIPT_DST"
  echo "Script copiado y se establecieron permisos en ${LAUNCHUP_SCRIPT_DST}"
else
  echo "Error: no se encontró ${LAUNCHUP_SCRIPT_SRC}"
  exit 1
fi

echo "Copiando el archivo de servicio minipc monitor & arduino..."
if [ -f "$LAUNCHUP_SERVICE_SRC" ]; then
  cp -i "$LAUNCHUP_SERVICE_SRC" "$LAUNCHUP_SERVICE_DST"
  echo "Servicio copiado a ${LAUNCHUP_SERVICE_DST}"
else
  echo "Error: no se encontró ${LAUNCHUP_SERVICE_SRC}"
  exit 1
fi

echo "Recargando la configuración de systemd..."
systemctl daemon-reload

echo "Habilitando el servicio minipc monitor & arduino..."
systemctl enable minipc-monitor-arduino.service

echo "Iniciando el servicio minipc monitor & arduino..."
systemctl start minipc-monitor-arduino.service

echo "El servicio de minipc monitor & arduino se ha instalado y está en ejecución."