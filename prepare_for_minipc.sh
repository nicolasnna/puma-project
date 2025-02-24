#!/bin/bash
echo "Borrando paquetes extras"
rm -rf puma_simulation
rm -rf puma_jetson
echo "instalando paquetes de python"
pip install backports.zoneinfo 
pip3 install backports.zoneinfo
echo "Terminado"