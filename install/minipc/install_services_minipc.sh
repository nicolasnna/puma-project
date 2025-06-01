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
  ["0"]="puma-roscore|${FILES_DIR}/puma-roscore.service|${SERVICE_DST}/puma-roscore.service"
  ["1"]="puma-imu-filter-realsense|${FILES_DIR}/puma-imu-filter-realsense.service|${SERVICE_DST}/puma-imu-filter-realsense.service|${FILES_DIR}/puma-imu-filter-realsense.sh|${SCRIPT_DST}/puma-imu-filter-realsense.sh"
  ["2"]="puma-localization-fusion|${FILES_DIR}/puma-localization-fusion.service|${SERVICE_DST}/puma-localization-fusion.service|${FILES_DIR}/puma-localization-fusion.sh|${SCRIPT_DST}/puma-localization-fusion.sh"
  ["3"]="puma-map-server|${FILES_DIR}/puma-map-server.service|${SERVICE_DST}/puma-map-server.service|${FILES_DIR}/puma-map-server.sh|${SCRIPT_DST}/puma-map-server.sh"
  ["4"]="puma-nav-manager|${FILES_DIR}/puma-nav-manager.service|${SERVICE_DST}/puma-nav-manager.service|${FILES_DIR}/puma-nav-manager.sh|${SCRIPT_DST}/puma-nav-manager.sh"
  ["5"]="puma-navigation|${FILES_DIR}/puma-navigation.service|${SERVICE_DST}/puma-navigation.service|${FILES_DIR}/puma-navigation.sh|${SCRIPT_DST}/puma-navigation.sh"
  ["6"]="puma-realsense|${FILES_DIR}/puma-realsense.service|${SERVICE_DST}/puma-realsense.service|${FILES_DIR}/puma-realsense.sh|${SCRIPT_DST}/puma-realsense.sh"
  ["7"]="puma-robot-status|${FILES_DIR}/puma-robot-status.service|${SERVICE_DST}/puma-robot-status.service|${FILES_DIR}/puma-robot-status.sh|${SCRIPT_DST}/puma-robot-status.sh"
  ["8"]="puma-state-machine|${FILES_DIR}/puma-state-machine.service|${SERVICE_DST}/puma-state-machine.service|${FILES_DIR}/puma-state-machine.sh|${SCRIPT_DST}/puma-state-machine.sh"
  ["9"]="puma-static-tf-map|${FILES_DIR}/puma-static-tf-map.service|${SERVICE_DST}/puma-static-tf-map.service|${FILES_DIR}/puma-static-tf-map.sh|${SCRIPT_DST}/puma-static-tf-map.sh"
  ["10"]="puma-visual-odometry|${FILES_DIR}/puma-visual-odometry.service|${SERVICE_DST}/puma-visual-odometry.service|${FILES_DIR}/puma-visual-odometry.sh|${SCRIPT_DST}/puma-visual-odometry.sh"
  ["11"]="puma-web-interface|${FILES_DIR}/puma-web-interface.service|${SERVICE_DST}/puma-web-interface.service|${FILES_DIR}/puma-web-interface.sh|${SCRIPT_DST}/puma-web-interface.sh"
  ["12"]="puma-minipc-monitor|${FILES_DIR}/puma-minipc-monitor.service|${SERVICE_DST}/puma-minipc-monitor.service|${FILES_DIR}/puma-minipc-monitor.sh|${SCRIPT_DST}/puma-minipc-monitor.sh"
  ["13"]="puma-arduino-nano|${FILES_DIR}/puma-arduino-nano.service|${SERVICE_DST}/puma-arduino-nano.service|${FILES_DIR}/puma-arduino-nano.sh|${SCRIPT_DST}/puma-arduino-nano.sh"
  ["14"]="puma-ip-manager|${FILES_DIR}/puma-ip-manager.service|${SERVICE_DST}/puma-ip-manager.service|${FILES_DIR}/puma-ip-manager.sh|${SCRIPT_DST}/puma-ip-manager.sh"
  ["15"]="puma-ip-cameras|${FILES_DIR}/puma-ip-cameras.service|${SERVICE_DST}/puma-ip-cameras.service|${FILES_DIR}/puma-ip-cameras.sh|${SCRIPT_DST}/puma-ip-cameras.sh"
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
  if [ -n "$script_src" ] && [ -f "$script_src" ]; then
    cp -iv "$script_src" "$script_dst" || exit 1
  else
    echo "Advertencia: Script definido pero no encontrado en $script_src"
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