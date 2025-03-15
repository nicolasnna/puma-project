import rospy
from puma_imu.read_topic import ReadMagnetometer
from puma_imu import utils
from sensor_msgs.msg import MagneticField
import numpy as np

if __name__ == "__main__":
  rospy.init_node("magnetometer_corrected")
  mag_reader = ReadMagnetometer("/mag")
  mag_corrected_pub = rospy.Publisher("/mag/corrected", MagneticField, queue_size=10)
  
  try:
    hard_iron_offset = rospy.get_param("~hard_iron_offset")
    soft_iron_offset = rospy.get_param("~soft_iron_correction")
  except KeyError:
    rospy.logerr("No se encontraron los parametros de calibración")
    exit()
  
  A_1 = np.array([soft_iron_offset['x'], soft_iron_offset['y'], soft_iron_offset['z']])
  b = np.array([hard_iron_offset['x'], hard_iron_offset['y'], hard_iron_offset['z']])
  
  while not rospy.is_shutdown():
    mx, my, mz = mag_reader.get_latest_magnetometer_data()
    mx_cal, my_cal, mz_cal = utils.estimate_new_magnetometer_value(A_1, b, mx, my, mz)
    mag_msg = MagneticField()
    mag_msg.header.stamp = rospy.Time.now()
    mag_msg.header.frame_id = "imu_link"
    mag_msg.magnetic_field.x = mx_cal
    mag_msg.magnetic_field.y = my_cal
    mag_msg.magnetic_field.z = mz_cal
    mag_corrected_pub.publish(mag_msg)
    rospy.Rate(30).sleep()