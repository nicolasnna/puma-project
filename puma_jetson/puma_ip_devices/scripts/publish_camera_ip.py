#!/usr/bin/env python3
from puma_ip_devices.utils_video_ip import AsyncVideoCapture, get_rtsp_url
import rospy
from sensor_msgs.msg import CompressedImage

def get_msg_img(data):
  msg = CompressedImage()
  msg.header.stamp = rospy.Time.now()
  msg.format = "jpeg"
  msg.data = data
  return msg
  
def main():
  rospy.init_node("video_stream_ip_node")
  user = rospy.get_param("~user", "admin")
  password = rospy.get_param("~password", "Admin1234")
  ip = rospy.get_param("~ip", "192.168.0.108")
  
  channels = rospy.get_param("~channels", [1])
  subtypes = rospy.get_param("~subtypes", [0])
  ns_camera = rospy.get_param("~ns_camera", "/puma/nvr")
  
  cameras = []
  publishes = []
  
  for idx, channel in enumerate(channels):
    subtype = subtypes[idx] if subtypes[idx] == 0 or subtypes[idx] == 1 else 0
    
    rtsp_url = get_rtsp_url(user, password, ip, channel, subtype)
    rospy.loginfo(f"RTSP URL para el canal {channel}: {rtsp_url}")
    camera = AsyncVideoCapture(rtsp_url)
    camera.start()
    cameras.append(camera)
    
    pub = rospy.Publisher(f"{ns_camera}/camera{idx}/image_raw/compressed", CompressedImage, queue_size=10)
    publishes.append(pub)
  
  rate = rospy.Rate(5)
  
  while not rospy.is_shutdown():
    for camera, pub in zip(cameras, publishes):
      frame = camera.get_encode_frame_to_jpeg()
      pub.publish(get_msg_img(frame))
    
    rate.sleep()
    
  for camera in cameras:
    camera.stop()
  
if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
  except Exception as e:
    rospy.logwarn(f"Error en el nodo: {e}")