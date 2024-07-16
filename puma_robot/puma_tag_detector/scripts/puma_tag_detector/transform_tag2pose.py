#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class TransformTag2Pose:
  ''' Class for transform april tag information to pose stamped '''
  def __init__(self, tag_name):
    self.tf_tag_ = tag_name
    self.output_frame = rospy.get_param('/tag_detector/output_frame', '/map')
    self.listener = tf.TransformListener()
    self.pose_pub = rospy.Publisher('/puma/tag_detector/pose'+self.tf_tag_, PoseStamped, queue_size=1)
    self.wait_duration = rospy.get_param('/tag_detector/wait_duration', 3.0)

  def publish_pose(self):
    ''' Get tf and publish pose relativ'''
    now = rospy.Time.now()
    publish = False
    try:
      self.listener.waitForTransform(self.output_frame, self.tf_tag_, now, rospy.Duration(self.wait_duration))
      trans, rot = self.listener.lookupTransform(self.output_frame, self.tf_tag_, now)
      publish = True
    except Exception as e:
      rospy.logerr_once("Error al obtener el valor de la transformada entre %s y %s", self.output_frame, self.tf_tag_)
      rospy.logerr(e)
    
    if publish:
      pose = PoseStamped()
      pose.header.stamp = rospy.Time.now()
      pose.header.frame_id = self.output_frame
      pose.pose.position.x = trans[0]
      pose.pose.position.y = trans[1]
      pose.pose.position.z = trans[2]
      pose.pose.orientation.x = rot[0]
      pose.pose.orientation.y = rot[1]
      pose.pose.orientation.z = rot[2]
      pose.pose.orientation.w = rot[3]

      self.pose_pub.publish(pose)