# Puma Tag Detector

## Resumen

Paquete ROS noetic para el procesado de la detección de tags. El nodo de este paquete trabaja de intermediario entre la detección del tag (con **`apriltag_ros`**) y el manejo de estados del robot (en **`puma_state_machine`**).

## Instalación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

    catkin build puma_tag_detector

#### Dependencias

- [Robot Operating System (ROS)](http://wiki.ros.org)
- [geometry_msgs](http://wiki.ros.org/geometry_msgs)
- [tf](http://wiki.ros.org/tf)
- [move_base_msgs](http://wiki.ros.org/move_base_msgs)
- [std_msgs](http://wiki.ros.org/std_msgs)
- [apriltag_ros](http://wiki.ros.org/apriltag_ros)

## Ejecución

El nodo puede ejecutarse directamente con:

    rosrun puma_tag_detector tag_detector_node.py

También se tiene dos archivos launch **`apriltag_detection.launch`** y **`tag_detector`**. El primero lanza ejecuta la detección de tag a partir del paquete **`apriltag_ros`**, mientras que el segundo se encarga de lanzar la transformación de los valores detectados para ser usados por los waypoints.

    roslaunch puma_tag_detector apriltag_detection.launch
    roslaunch puma_tag_detector tag_detector.launch

## Launch

- **apriltag_detection.launch:** Lanzador para iniciar el detector de april tags. Este lanzador carga las configuraciones de **`settings.yaml`** y **`tags.yaml`**. Además tiene parametros y remap del nodo:

  - **`remap`**

    - **`image_rect`** ⇒ **`/puma/camera/color/image_raw`**
    - **`camera_info`** ⇒ **`/puma/camera/color/camera_info`**

  - **`param`**

    - **`camera_frame`** (String, valor: camera_link)
    - **`publish_tag_detections_image`** (Bool, valor: false)

- **tag_detector.launch:** Lanzador para el nodo **`tag_detector_node`**. Adicionalmente carga el archivo **`tag_detector_params.yaml`**.

## Config files

- **settings.yaml**

  - **`tag_family`** (String, valor: "tag36h11")

  - **`tag_threads`** (Int, valor: 2)
  - **`tag_decimate`** (Float, valor: 1.0)
  - **`tag_blur`** (Float, valor: 0.0)
  - **`tag_refine_edges`** (Int, valor: 1)
  - **`tag_debug`** (Int, valor: 0)
  - **`max_hamming_dist`** (Int, valor: 2)
  - **`publish_tf`** (Bool, valor: true)
  - **`transport_hint`** (String, valor: "raw")

- **tags.yaml**

  - **`standalone_tags`** (Array-Object)

    - {id: 0, size: 0.1, name: tag_0}

      Se necesita fijar tanto el `id` como el `size` en metros del tag físico, para obtener una correcta estimación de posición.

- **tag_detector_params.yaml**

  - **`tag_detector`**

    - **`~tag_names`** (Array-String, valor: [/tag_0])

    - **`~output_frame`** (String, valor: "/map")
    - **`~camera_frame`** (String, valor: "/camera_link")
    - **`~wait_duration`** (Float, valor: 3.0)
    - **`~enable_topic`** (String, valor: "/puma/tag_detector/enable")

## Nodos

### tag_detector

Se encarga de transformar la posición detectada del tag, a una posición de destino (**`move_base_goal`**). Este nodo tiene un suscriptor para activar o desactivar el nodo. La obtención de posición del tag se realiza a partir de transformadas de marco de coordenadas.

#### Suscriptor

- **`/puma/tag_detector/pose/<tag_name>`** (geometry_msgs/PoseStamped)

  Los `tag_name` depende del parámetro colocado. (Ej.: `tag_0`, `tag_1`)

- **`/puma/tag_detector/enable`** (std_msgs/Bool)

  Habilita la activación del nodo.

#### Publicador

- **`/puma/tag_detector/pose/<tag_name>`** (geometry_msgs/PoseStamped)

  Publica la posición del tag.

- **`/puma/tag_detector/pose2camera/<tag_name>`** (geometry_msgs/PoseStamped)

  Publica la posición relativa del tag respecto a la posición de la cámara.

- **`/puma/tag_detector/goal/<tag_name>`** (move_base_msgs/MoveBaseGoal)

  Publica la posición de destino respecto a la posición del tag.

#### Parámetros

- **`tag_detector`**

  - **`~tag_names`** (Array-String, default: ["/tag_0"])

    Tags a considerar para la obtención de posición.

  - **`~output_frame`** (String, default: "/map")

    Marco de coordenadas de la posición de destino calculada.

  - **`~camera_frame`** (String, default: "/camera_link")

    Marco de coordenadas de referencia para la obtención de la posición relativa.

  - **`~wait_duration`** (Float, default: 3.0)

    Tiempo de espera para la obtención de la transformada de coordenadas,

  - **`~enable_topic`** (String, default: "/puma/tag_detector/enable")

    Tópico de suscripción para la activación del nodo.

  - **`~distance_to_tag`** (Float, default: 6.0)

    Distancia a dejar entre la ubicación de destino y la posición del tag.
