# Puma Global Planner

## Resumen

Planificador global para ser utilizado con `move_base` como plugin. Emplea una busqueda de tipo A* para encontrar la mejor ruta, considerando el uso de celdas para la busqueda. Luego implementa un postprocesado usando curvas de dubins para optimizar la ruta para el robot Puma.

## Dependencias

- [costmap_2d](http://wiki.ros.org/costmap_2d)
- [geometry_msgs](http://wiki.ros.org/geometry_msgs)
- [nav_core](http://wiki.ros.org/nav_core)
- [pluginlib](http://wiki.ros.org/pluginlib)
- [nav_msgs](http://wiki.ros.org/nav_msgs)
- [tf](http://wiki.ros.org/tf)
- [roscpp](http://wiki.ros.org/roscpp)
- [visualization_msgs](http://wiki.ros.org/visualization_msgs)
- Dynamic Reconfigure

## Compilación

Se realiza compilación por [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/):

```bash
    catkin build puma_global_planner
```

## Incorporar en move_base

Para usarlo como planificador global es necesario definir el parámetro **base_global_planner**:

```xml
    <param name="base_global_planner" value="puma_global_planner/PumaGlobalPlanner" />
```

## puma_global_planner

Este planificador se encarga de plantear la trayectoria global usando el algoritmo A* para la busqueda de la ruta más corta. Para considerar las limitaciones en el giro del robot, se incluye un postprocesado de la ruta usando curva de dubins. Este planificador puede considerar como obstáculo cualquier celda ocupada.

### Publicador

- **`move_base/PumaGlobalPlanner/global_plan`** (nav_msgs/Path)

  Publica la trayectoria generada con A*.

- **`move_base/PumaGlobalPlanner/global_plan_dubin`** (nav_msgs/Path)

  Publica la trayectoria postprocesada con las curvas de dubins.

### Parámetros

- **`resolution`** (double, default: 0.5)

  Resolución del tamaño de las celdas a considerar para la busqueda de ruta óptima, en metros.

- **`turning_radius`** (double, default: 2.5)

  Radio de giro a considerar para la generación de las curvas de dubins.

- **`meters_subsamples`** (double, default: 7.0)

  Distancia en metros a considerar para postprocesar la curva dubins. Con distancias más grandes evita generar curvas de más.

- **`step_size_dubins`** (double, default: 0.3)

  Tamaño de paso para la generación de curvas en metros.

- **`use_dubins`** (boolean, default: true)

  Habilita del uso de la curva generada con dubins para ser usado por el planificador local.

- **`detect_obstacles`** (boolean, default: false)

  Habilita la consideración de obstáculos para la generación de trayectoria.