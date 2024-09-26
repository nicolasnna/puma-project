#!/usr/bin/env python3
import rospy
from puma_tag_detector.transform_tag2pose import TransformTag2Pose
from puma_tag_detector.goal_from_posetag import GoalFromPoseTag
from std_msgs.msg import Bool

def callback_activate_detector(is_activate):
  global enable
  # if is_activate.data:
  #   enable = not enable
  enable = is_activate.data

if __name__ == "__main__":
  rospy.init_node("tag_detector_node")
  global enable
  enable = False
  # Subscriber
  enable_topic = rospy.get_param('/tag_detector/enable_topic', '/puma/tag_detector/enable')
  rospy.Subscriber(enable_topic, Bool, callback_activate_detector)
  # Array with multiples classes 
  tag_names = rospy.get_param('/tag_detector/tag_names',['/tag_0'])
  transform_tags = []
  goal_tag = []
  for tag in tag_names:
    transform_tags.append(TransformTag2Pose(tag))
    goal_tag.append(GoalFromPoseTag(tag))
    
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    if enable:
      for transforms in transform_tags:
        transforms.publish_pose()
    
    rate.sleep()