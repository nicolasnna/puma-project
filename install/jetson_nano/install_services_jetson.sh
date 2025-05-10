#!/bin/bash

# Verificar root
if [ "$EUID" -ne 0 ]; then
  echo "Por favor, ejecuta el script como root"
  exit 1
fi

# Directorio del script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FILES_DIR="${SCRIPT_DIR}/files"
SERVICE_DST="/etc/systemd/system"
SCRIPT_DST="/usr/local/bin"

# Decarar servicios en arreglo tipo:
# "clave" = "nombre | ruta_servicio_origen | ruta_servicio_destino | ruta_script_origen | ruta_script_destino"
declare -A servicios=(
  ["1"]="puma-arduino-mega|${FILES_DIR}/puma-arduino-mega.service|${SERVICE_DST}/puma-arduino-mega.service|${FILES_DIR}/puma-arduino-mega.sh|${SCRIPT_DST}/puma-arduino-mega.sh"
  ["2"]="puma-control-mode|${FILES_DIR}/puma-control-mode.service|${SERVICE_DST}/puma-control-mode.service|${FILES_DIR}/puma-control-mode.sh|${SCRIPT_DST}/puma-control-mode.sh"
  ["3"]="puma-controller|${FILES_DIR}/puma-controller.service|${SERVICE_DST}/puma-controller.service|${FILES_DIR}/puma-controller.sh|${SCRIPT_DST}/puma-controller.sh"
  ["4"]="puma-gps|${FILES_DIR}/puma-gps.service|${SERVICE_DST}/puma-gps.service|${FILES_DIR}/puma-gps.sh|${SCRIPT_DST}/puma-gps.sh"
  ["5"]="puma-imu-bringup|${FILES_DIR}/puma-imu-bringup.service|${SERVICE_DST}/puma-imu-bringup.service|${FILES_DIR}/puma-imu-bringup.sh|${SCRIPT_DST}/puma-imu-bringup.sh"
  ["6"]="puma-jetson-monitor|${FILES_DIR}/puma-jetson-monitor.service|${SERVICE_DST}/puma-jetson-monitor.service|${FILES_DIR}/puma-jetson-monitor.sh|${SCRIPT_DST}/puma-jetson-monitor.sh"
  ["7"]="puma-joy|${FILES_DIR}/puma-joy.service|${SERVICE_DST}/puma-joy.service|${FILES_DIR}/puma-joy.sh|${SCRIPT_DST}/puma-joy.sh"
  ["8"]="puma-odom-visual|${FILES_DIR}/puma-odom-visual.service|${SERVICE_DST}/puma-odom-visual.service|${FILES_DIR}/puma-odom-visual.sh|${SCRIPT_DST}/puma-odom-visual.sh"
  ["9"]="puma-parking|${FILES_DIR}/puma-parking.service|${SERVICE_DST}/puma-parking.service|${FILES_DIR}/puma-parking.sh|${SCRIPT_DST}/puma-parking.sh"
  ["10"]="puma-realsense|${FILES_DIR}/puma-realsense.service|${SERVICE_DST}/puma-realsense.service|${FILES_DIR}/puma-realsense.sh|${SCRIPT_DST}/puma-realsense.sh"
  ["11"]="puma-reverse|${FILES_DIR}/puma-reverse.service|${SERVICE_DST}/puma-reverse.service|${FILES_DIR}/puma-reverse.sh|${SCRIPT_DST}/puma-reverse.sh"
  ["12"]="puma-ip-cameras|${FILES_DIR}/puma-ip-cameras.service|${SERVICE_DST}/puma-ip-cameras.service|${FILES_DIR}/puma-ip-cameras.sh|${SCRIPT_DST}/puma-ip-cameras.sh"
  ["13"]="puma-rosbridge-server|${FILES_DIR}/puma-rosbridge-server.service|${SERVICE_DST}/puma-rosbridge-server.service|${FILES_DIR}/puma-rosbridge-server.sh|${SCRIPT_DST}/puma-rosbridge-server.sh"
  ["14"]="puma-ip-manager|${FILES_DIR}/puma-ip-manager.service|${SERVICE_DST}/puma-ip-manager.service|${FILES_DIR}/puma-ip-manager.sh|${SCRIPT_DST}/puma-ip-manager.sh"
)

mostrar_menu() {
  echo "Servicios disponibles para instalar:"
  for key in $(echo "${!servicios[@]}" | tr ' ' '\n' | sort -n); do
    IFS='|' read -r nombre _ _ _ _ <<< "${servicios[$key]}"
    echo " $key) $nombre"
  done
}

instalar_servicio() {
  local id=$1
  IFS='|' read -r nombre servicio_src servicio_dst script_src script_dst <<< "${servicios[$id]}"

  echo -e "\nInstalando $nombre..."

  #copiar servicio
  if [ -f "$servicio_src" ]; then
    cp -iv "$servicio_src" "$servicio_dst" || exit 1
  else
    echo "Error: no se encontro $servicio_src"
    return 1
  fi

  #copiar script
  if [ -f "$script_src" ]; then
    cp -iv "$script_src" "$script_dst" || exit 1
  else
    echo "Error: no se encontro $script_src"
    return 1
  fi

  return 0
}

# Mostrar menu
mostrar_menu

# Solicitar seleccion
read -rp "Selecciona los servicios a instalar (separados por espacios): " seleccion

# Procesar instalacion
declare -a servicios_instalados
for id in $seleccion; do
  if [[ -n "${servicios[$id]}" ]]; then
    if instalar_servicio "$id"; then
      servicios_instalados+=("${servicios[$id]%%|*}")
    fi
  else
    echo "Opcion invalida: $id"
  fi
done

# Recargar y activar servicios
if [ ${#servicios_instalados[@]} -gt 0 ]; then
  echo -e "\nRecargando configuración de systemd..."
  systemctl daemon-reload

  for servicio in "${servicios_instalados[@]}"; do
      echo "Habilitando $servicio..."
      systemctl enable "$servicio.service"
      echo "Iniciando $servicio..."
      systemctl start "$servicio.service"
  done
fi