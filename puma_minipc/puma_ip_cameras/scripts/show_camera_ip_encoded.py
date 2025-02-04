import rospy
from puma_ip_cameras.camera_stream_track import CameraStreamTrack
import cv2

async def show_camera(cameraInstance):
  cv2.imshow('VIDEO', cameraInstance.recv())
  cv2.waitKey(1)
  

if __name__ == '__main__':
  rospy.init_node('ip_camera_node')
  rtsp_url = "rtsp://admin:smartbot2023@10.42.0.103:554/cam/realmonitor?channel=1&subtype=0"
  
  camera1 = CameraStreamTrack(rtsp_url)
  rospy.loginfo('Iniciando captura de video...')
  while not rospy.is_shutdown():
    print(camera1.recv())
    rospy.sleep(0.1)  