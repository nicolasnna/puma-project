#!/bin/bash
# Este script debe ejecutarse con privilegios de root.

# Verifica que el script se ejecute como root
if [ "$EUID" -ne 0 ]; then
  echo "Por favor, ejecuta el script como root."
  exit 1
fi

# Definir rutas de origen (asumiendo que los archivos están en el mismo directorio que el script)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_SRC="${SCRIPT_DIR}/boot/puma-jetson.service"
LAUNCHUP_SRC="${SCRIPT_DIR}/boot/puma-bringup-ros.sh"

# Definir destinos
SERVICE_DST="/etc/systemd/system/puma-jetson.service"
LAUNCHUP_DST="/usr/local/bin/puma-bringup-ros.sh"

echo "Copiando el archivo de servicio..."
if [ -f "$SERVICE_SRC" ]; then
    cp -i "$SERVICE_SRC" "$SERVICE_DST"
    echo "Servicio copiado a ${SERVICE_DST}"
else
    echo "Error: no se encontró ${SERVICE_SRC}"
    exit 1
fi

echo "Copiando el script de bringup..."
if [ -f "$LAUNCHUP_SRC" ]; then
    cp -i "$LAUNCHUP_SRC" "$LAUNCHUP_DST"
    # Asegurar que el script sea ejecutable
    chmod +x "$LAUNCHUP_DST"
    echo "Script copiado y se establecieron permisos en ${LAUNCHUP_DST}"
else
    echo "Error: no se encontró ${LAUNCHUP_SRC}"
    exit 1
fi

echo "Recargando la configuración de systemd..."
systemctl daemon-reload

echo "Habilitando el servicio puma-jetson..."
systemctl enable puma-jetson.service

echo "Iniciando el servicio puma-jetson..."
systemctl start puma-jetson.service

echo "Instalación completada. El servicio puma-jetson se ha instalado, habilitado e iniciado."