#! /usr/bin/env python3

import rospy
import actionlib
import puma_navigation.msg

class RelocateAction:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('puma_relocate', )