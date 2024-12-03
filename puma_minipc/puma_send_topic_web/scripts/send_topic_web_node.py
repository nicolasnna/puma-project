import rospy
from puma_send_topic_web.resend_topic import ResendTopic
from sensor_msgs.msg import CompressedImage, NavSatFix
from nav_msgs.msg import Odometry
from puma_msgs.msg import StatusArduino, GoalGpsNavInfo, Log
import time

if __name__ == '__main__':
  rospy.init_node("send_topic_web")
  camera_realsense = ResendTopic('/puma/camera/color/image_raw/compressed', CompressedImage, 5, '/puma/web/camera_realsense')
  odometry = ResendTopic('/puma/odometry/filtered', Odometry, 2, '/puma/web/odometry')
  gps = ResendTopic('/puma/sensors/gps/fix', NavSatFix, 1, '/puma/web/gps')
  arduino_status = ResendTopic('/puma/arduino/status', StatusArduino, 1, '/puma/web/arduino_status')
  nav_gps = ResendTopic('/puma/waypoints/gps_nav_info', GoalGpsNavInfo, 1, '/puma/web/gps_nav_info')
  
  log_publish = rospy.Publisher('/puma/logs/add_log', Log, queue_size=1)
  
  rospy.loginfo("Realizando envio de topicos a la web")
  log_msg = Log()
  log_msg.level = 0
  log_msg.node = rospy.get_name()
  log_msg.content = "Nodo activo. Reenviando los topicos a la web."
  time.sleep(0.1)
  log_publish.publish(log_msg)
  
  try: 
    while not rospy.is_shutdown():
      # Publish Resends topics
      camera_realsense.publish()
      odometry.publish()
      gps.publish()
      arduino_status.publish()
      nav_gps.publish()
      
      rospy.Rate(15).sleep()
      
  except Exception as e:
    rospy.logwarn(f"El nodo {rospy.get_name()} se ha cerrado debido al error: {e}")