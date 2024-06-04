import rospy
from std_msgs.msg import Float32
from puma_odometry.calculate_odometry import CalculateOdometry
if __name__ == '__main__':
  try:
    rospy.init_node('puma_odomety_node')
    odometry = CalculateOdometry()
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
      odometry.calculate_odometry()
      rate.sleep()
      
  except rospy.ROSInterruptException:
    rospy.logwarn("Nodo 'puma_odometry' Desactivado!!!")
    
  except rospy.ROSException:
    rospy.logerr("Error al iniciar el nodo 'puma_odometry'")