#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
import tf
import tf2_ros
import geometry_msgs.msg

class OdomTransfromInit():
    def __init__(self):

        # tf broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def publish_tf_transform(self):
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id =  "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0
        # Set the orientation (quaternion) - here assuming no initial rotation
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
  

        self.tf_broadcaster.sendTransform(t)