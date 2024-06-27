#!/usr/bin/env python3
import rospy
import smach

class PathComplete(smach.State):
  """ State when is completed path planned """
  def __init__(self):
    smach.State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])
    
  def execute(self, ud):
    rospy.loginfo('###############################')
    rospy.loginfo('##### REACHED FINISH GATE #####')
    rospy.loginfo('###############################')
    return 'success'