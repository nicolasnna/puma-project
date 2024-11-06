#!/usr/bin/env python3
import rospy
from puma_waypoints.path_select import PathSelect
from puma_waypoints.path_follow import PathFollow
from puma_waypoints.path_complete import PathComplete
from puma_waypoints.charge_mode import ChargeMode
import smach
import smach_ros

if __name__ == "__main__":
  rospy.init_node('puma_waypoints')
  sm = smach.StateMachine(outcomes=['success'])
  
  with sm:
    smach.StateMachine.add('PATH_SELECT',PathSelect(),
                           transitions={'path_follow_mode':'PATH_FOLLOW', 'charge_mode':'CHARGE_MODE'})
    smach.StateMachine.add('PATH_FOLLOW',PathFollow(),
                           transitions={'success':'PATH_COMPLETE', 'aborted':'PATH_SELECT'}) 
    smach.StateMachine.add('PATH_COMPLETE',PathComplete(),
                           transitions={'select_plan':'PATH_SELECT', 'start_plan':'PATH_FOLLOW', 'charge_mode':'CHARGE_MODE'})
    smach.StateMachine.add('CHARGE_MODE', ChargeMode(),
                           transitions={'finish_charge':'PATH_SELECT'})
    
  # For see state machine in diagram
  sis = smach_ros.IntrospectionServer('puma', sm, '/WAYPOINTS')
  sis.start()
  
  while not rospy.is_shutdown():
    outcome = sm.execute()
    sis.stop()
