#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
import tf
import tf2_ros
import geometry_msgs.msg

class InitialOdomPublisher:
    def __init__(self):
        rospy.init_node('initial_odom_publisher')

        # Publisher for odometry
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        # tf broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Create and publish initial odometry
        self.publish_initial_odom()

    def publish_initial_odom(self):
        # Create the odometry message
        odom_msg = Odometry()

        # Fill in the header
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set the position
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0

        # Set the orientation (quaternion) - here assuming no initial rotation
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        # Set the covariance - here assuming no uncertainty
        odom_msg.pose.covariance = [0]*36

        # Set the velocity - here assuming no initial velocity
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        # Set the covariance - here assuming no uncertainty
        odom_msg.twist.covariance = [0]*36

        # Publish the message
        self.odom_pub.publish(odom_msg)
        rospy.loginfo("Initial odometry published.")

        # Publish the tf transformation
        self.publish_tf_transform(odom_msg)

    def publish_tf_transform(self, odom_msg):
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = odom_msg.header.frame_id
        t.child_frame_id = odom_msg.child_frame_id

        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    try:
        odometry = InitialOdomPublisher()
        while not rospy.is_shutdown():
          odometry.publish_initial_odom()
          rospy.Rate(1).sleep()
        
    except rospy.ROSInterruptException:
        pass
