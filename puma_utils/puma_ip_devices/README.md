# Puma Ip Devices

## Resumen

Paquete para la lectura de imagenes desde las cámaras IP, las transforma en jpeg y las publica en un tópico de ROS.

## Dependencias

- rospy
- base64
- cv2

## Instalación

Emplear catkin tools para la compilación e instalación:

    catkin build puma_ip_cameras

## Ejecución 

Se tiene definido un lanzador para el nodo principal `puma_ip_cameras_node`:

    roslaunch puma_ip_cameras publish_camera_nvr.launch

Dentro del lanzador se define los parámetros, canales y tipo de transmisión para la captura de imagen desde un nvr (Network Video Recorder) o desde una cámara IP.

Desde un NVR se puede capturar múltiples canales a la vez.

## Scripts

### publish_camera_ip.py

Conecta con una cámara ip o nvr usando rtsp con el puerto 554, obtiene la imagen convirtiendola en jpeg y las publica en un tópico de ROS.

#### **Parámetros**

- `ip` (default: "192.168.0.108", type: String)

  Dirección IP de la cámara o NVR.
   
- `user` (default: admin, type: String)  
  
  Usuario para la conexión.

- `password` (default: Admin1234, type: String)

  Contraseña para la conexión.

- `channels` (default: [1], type: List, Numbers)

  Lista de canales a capturar desde la misma ip.

- `subtypes` (default: [0], type: List, Numbers)

  Lista del tipo de transmisión a capaturar de cada canal, 0 para tansmisión principal y 1 para transmisión secundaria.

- `ns_camera` (default: "/puma/nvr", type: String)

  Espacio de nombres para la publicación de tópicos.

#### **Publicadores**

  - `<ns_camera>/camera_<index>/image_raw/compressed` (sensor_msgs/CompressedImage)

    Multiples publicadores de la imagen en jpeg según la cantidad de canales definidos.