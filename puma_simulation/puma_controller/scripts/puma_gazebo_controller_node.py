#!/usr/bin/env python
import rospy
from std_msgs import Float64

_pub_dir_left = rospy.Publisher('/dir_left_controller/command', Float64, queue_size=5)
_pub_dir_right = rospy.Publisher('/dir_right_controller/command', Float64, queue_size=5)
