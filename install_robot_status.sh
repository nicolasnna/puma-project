#!/bin/bash

if [ "$EUID" -ne 0 ]; then
  echo "Por favor, ejecuta el script como root."
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCHUP_SCRIPT_SRC="${SCRIPT_DIR}/boot/puma-robot-status.sh"
LAUNCHUP_SERVICE_SRC="${SCRIPT_DIR}/boot/puma-robot-status.service"

# Definir destinos
LAUNCHUP_SCRIPT_DST="/usr/local/bin/puma-robot-status.sh"
LAUNCHUP_SERVICE_DST="/etc/systemd/system/puma-robot-status.service"

echo "Copiando el script de robot status..."
if [ -f "$LAUNCHUP_SCRIPT_SRC" ]; then
  cp -i "$LAUNCHUP_SCRIPT_SRC" "$LAUNCHUP_SCRIPT_DST"
  # Asegurar que el script sea ejecutable
  chmod +x "$LAUNCHUP_SCRIPT_DST"
  echo "Script copiado y se establecieron permisos en ${LAUNCHUP_SCRIPT_DST}"
else
  echo "Error: no se encontró ${LAUNCHUP_SCRIPT_SRC}"
  exit 1
fi

echo "Copiando el archivo de servicio robot status..."
if [ -f "$LAUNCHUP_SERVICE_SRC" ]; then
  cp -i "$LAUNCHUP_SERVICE_SRC" "$LAUNCHUP_SERVICE_DST"
  echo "Servicio copiado a ${LAUNCHUP_SERVICE_DST}"
else
  echo "Error: no se encontró ${LAUNCHUP_SERVICE_SRC}"
  exit 1
fi

echo "Recargando la configuración de systemd..."
systemctl daemon-reload

echo "Habilitando el servicio robot status..."
systemctl enable puma-robot-status.service

echo "Iniciando el servicio robot status..."
systemctl start puma-robot-status.service

echo "El servicio de robot status se ha instalado y está en ejecución."