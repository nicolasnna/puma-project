#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros
import tf

class FrameMapOdom():
  '''
  Publish frame map using gps enu and Imu
  '''
  def __init__(self):
    # Get params from rosparam
    topic_imu = rospy.get_param('~frame_map_odom/topic_imu', 'puma/sensors/imu/raw')
    topic_gps_enu = rospy.get_param('~frame_map_odom/topic_gps_enu', 'puma/sensors/gps/enu')
    
    # Subscriber and publisher
    rospy.Subscriber(topic_imu, Imu, self.imu_callback)
    rospy.Subscriber(topic_gps_enu, PoseWithCovarianceStamped, self.gps_enu_callback)
    
    # Varible
    self.orientation = [0, 0, 0, 0]
    self.map_broadcaster = tf2_ros.TransformBroadcaster()
    
  def imu_callback(self, imu_data):
    '''
    Callback from data imu
    '''
    self.orientation[0] = imu_data.orientation.x
    self.orientation[1] = imu_data.orientation.y
    self.orientation[2] = imu_data.orientation.z
    self.orientation[3] = imu_data.orientation.w
  
  def gps_enu_callback(self, enu_data):
    '''
    Callback from gps enu
    '''
    x = enu_data.pose.pose.position.x
    y = enu_data.pose.pose.position.y
    z = enu_data.pose.pose.position.z
    
    self.publish_transform(x, y, z)
    
  def publish_transform(self, x, y, z):
    '''
    Publish transform between map base link
    '''
    current_time = rospy.Time.now()
    
    listener_tf = tf.TransformListener()
    try:
      listener_tf.waitForTransform("odom", "base_link", rospy.Time(0), rospy.Duration(1.0))
      trans_tf, rot_tf = listener_tf.lookupTransform("odom", "base_link", rospy.Time(0))
    except Exception as e:
      rospy.logerr('Error al obtener la lectura de las tf: %s', e)
    
    t = TransformStamped()
    t.header.stamp = current_time
    t.header.frame_id = 'map'
    t.child_frame_id = 'odom'
    
    t.transform.translation.x = x - trans_tf[0]
    t.transform.translation.y = y - trans_tf[1]
    t.transform.translation.z = z - trans_tf[2]
    
    t.transform.rotation.x = self.orientation[0]
    t.transform.rotation.y = self.orientation[1]
    t.transform.rotation.z = self.orientation[2]
    t.transform.rotation.w = self.orientation[3]
    
    self.map_broadcaster.sendTransform(t)