#!/usr/bin/env python3
import rospy
import smach
import threading
import time
from std_msgs.msg import Empty

class PathComplete(smach.State):
  """ State when is completed path planned """
  def __init__(self):
    smach.State.__init__(self, outcomes=['start_plan', 'select_plan','charge_mode'], input_keys=['waypoints'], output_keys=['waypoints'])
    self.ns_robot = '/puma/waypoints'
    
  def execute(self, ud):
    rospy.loginfo('###############################')
    rospy.loginfo('##### REACHED FINISH GATE #####')
    rospy.loginfo('###############################')
    self.start_plan = False
    self.select_plan = False
    self.charge_mode = False
    
    rospy.loginfo("Waiting for change to start, select or charge car mode")
    
    # Thread for start plan again
    def wait_for_init_plan():
      rospy.wait_for_message(self.ns_robot+'/plan_ready', Empty)
      rospy.loginfo("Start plan again")
      self.start_plan = True
    wait_for_init_thread = threading.Thread(target=wait_for_init_plan, daemon=True)
    wait_for_init_thread.start()
    
    # Thread for reset and select plan
    def wait_for_select_plan():
      rospy.wait_for_message(self.ns_robot+'/plan_reset', Empty)
      rospy.loginfo("Change to select plan mode")
      self.select_plan = True
    wait_for_select_thread = threading.Thread(target=wait_for_select_plan, daemon=True)
    wait_for_select_thread.start()
    
    # Thread for charge car mode
    def wait_for_charge_mode():
      rospy.wait_for_message(self.ns_robot+'/run_charge_mode',Empty)
      rospy.loginfo("Change to charge car mode")
      self.charge_mode = True
    wait_for_charge_thread = threading.Thread(target=wait_for_charge_mode, daemon=True)
    wait_for_charge_thread.start()

    # Loop for waiting message
    while not rospy.is_shutdown():
      time.sleep(0.5)
      
      if self.select_plan:
        return 'select_plan'
      elif self.start_plan:
        return 'start_plan'
      elif self.charge_mode:
        return 'charge_mode'