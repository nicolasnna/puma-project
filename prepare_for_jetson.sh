#!/bin/bash

echo "Borrando paquetes extras"
rm -rf puma_arduino
rm -rf puma_simulation
rm -rf puma_discarded
rm -rf puma_minipc
echo "Cambiando permisos a los scripts"
source puma_jetson/make_exec.sh || echo "No se encuentra 'make_exec.sh'"
echo "Terminado"