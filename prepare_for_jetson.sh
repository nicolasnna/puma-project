#!/bin/bash

echo "Borrando paquetes extras"
rm -rf puma_arduino
rm -rf puma_simulation
rm -rf puma_discarded
rm -rf puma_minipc
echo "Cambiando permisos a los scripts"

# Definir ruta absoluta del script actual
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Llamar al script make_exec.sh usando ruta absoluta
source "$SCRIPT_DIR/puma_jetson/make_exec.sh" || echo "No se encuentra 'make_exec.sh'"

echo "Terminado"