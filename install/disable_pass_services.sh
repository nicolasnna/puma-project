#!/bin/bash

if [ "$EUID" -ne 0 ]; then
  echo "Por favor, ejecuta el script como root"
  exit 1
fi

USUARIO="$SUDO_USER"
SUDOERS_FILE="/etc/sudoers.d/puma_services"
PREFIX="puma"

COMMANDS=(
  "/bin/systemctl start ${PREFIX}-*"
  "/bin/systemctl stop ${PREFIX}-*"
  "/bin/systemctl restart ${PREFIX}-*"
  "/bin/systemctl status ${PREFIX}-*"
  "/bin/systemctl enable ${PREFIX}-*"
  "/bin/systemctl disable ${PREFIX}-*"
  "/bin/systemctl reload ${PREFIX}-*"
  "/bin/systemctl daemon-reload"
)

sudo touch "$SUDOERS_FILE"
for cmd in "${COMMANDS[@]}"; do
  if ! sudo grep -q "$USUARIO ALL=(ALL) NOPASSWD: $cmd" "$SUDOERS_FILE"; then
    echo "$USUARIO ALL=(ALL) NOPASSWD: $cmd" | sudo tee -a "$SUDOERS_FILE"
  fi
done

if sudo visudo -c -f "$SUDOERS_FILE"; then
  echo "Configuración de sudoers actualizada correctamente para $USUARIO."
else
  echo "Error al actualizar la configuración de sudoers."
  exit 1
fi