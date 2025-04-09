import rospy
from puma_ip_cameras.utils_video_ip import AsyncVideoCapture, get_rtsp_url
import cv2

if __name__ == '__main__':
  rospy.init_node('ip_camera_node')
  # rtsp_url = "rtsp://admin:smartbot2023@10.42.0.103:554/cam/realmonitor?channel=1&subtype=0"
  # rtsp_url = "rtsp://admin:Admin1234@192.168.0.108:554/cam/realmonitor?channel=2&subtype=1"
  rtsp_url = get_rtsp_url("admin", "Admin1234", "192.168.0.108", 2, 1)
  
  camera1 = AsyncVideoCapture(rtsp_url)
  rospy.loginfo('Iniciando captura de video...')
  camera1.start()
  while not rospy.is_shutdown():
    frame = camera1.get_frame()
    
    if frame is not None:
      _, encoded_frame = cv2.imencode('.jpg', frame)
      
      # Decodifica el frame para visualizarlo
      decoded_frame = cv2.imdecode(encoded_frame, cv2.IMREAD_COLOR)
      # Muestra el video
      cv2.imshow('VIDEO', decoded_frame)
      if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    else:
      rospy.logwarn("No se pudo obtener el frame.")
    rospy.sleep(0.1)  # Pequeña pausa para evitar consumo excesivo de CPU
    
  camera1.stop()
  cv2.destroyAllWindows()