import rospy
from puma_ip_cameras.utils import send_video_to_backend
import threading
import asyncio

def start_asyncio_task(rtsp_url, backend_url, camera_name):
  asyncio.run(send_video_to_backend(rtsp_url, backend_url, camera_name))
  
if __name__ == "__main__":
  rospy.init_node("video_stream_node")
  
  rtsp_url = "rtsp://admin:smartbot2023@10.42.0.103:554/cam/realmonitor?channel=1&subtype=0"
  backend_url = "ws://10.42.0.22:8000/ipcam/ws"
  camera_name = "robot_camera"

  # Crear un hilo para ejecutar la tarea asyncio
  thread = threading.Thread(target=start_asyncio_task, args=(rtsp_url, backend_url, camera_name))
  thread.start()

  rospy.loginfo("Nodo de streaming iniciado")
  while not rospy.is_shutdown():
    rospy.spin()  # Mantener el nodo funcionando para procesar mensajes de ROS

  # Esperar a que el hilo termine antes de cerrar
  thread.join()