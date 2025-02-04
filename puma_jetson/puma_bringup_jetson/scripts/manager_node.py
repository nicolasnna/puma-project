#!/usr/bin/env python3
import rospy
import roslaunch
import time

def start_rosserial():
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    
    # Definir el nodo rosserial_python directamente en el código
    node = roslaunch.core.Node(
        package='rosserial_python',
        node_type='serial_node.py',
        name='rosserial_node',
        output='screen',
        args='/dev/ttyUSB0 _baud:=250000'
    )
    
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)
    return process, launch

def main():
    rospy.init_node('rosserial_arduino_manager')
    process, launch = start_rosserial()
    
    try:
        while not rospy.is_shutdown():
            if not process.is_alive():
                rospy.logwarn("El nodo rosserial se ha cerrado. Reiniciando...")
                process.stop()  # Detener el proceso anterior
                process, launch = start_rosserial()
            time.sleep(1)  # Esperar un segundo antes de volver a verificar
    except rospy.ROSInterruptException:
        pass
    finally:
        process.stop()  # Asegurarse de cerrar el proceso al finalizar
        launch.shutdown()  # Asegurarse de cerrar el lanzador al finalizar

if __name__ == '__main__':
    main()