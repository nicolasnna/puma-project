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
    smach.StateMachine.add('GET_PATH',PathSelect(),
                           transitions={'success':'FOLLOW_PATH', 'charge_mode':'CHARGE_MODE'},
                           remapping={'waypoints':'waypoints'})
    smach.StateMachine.add('FOLLOW_PATH',PathFollow(),
                           transitions={'success':'COMPLETE_PATH', 'aborted':'GET_PATH'}, 
                           remapping={'waypoints':'waypoints'}) 
    smach.StateMachine.add('COMPLETE_PATH',PathComplete(),
                           transitions={'select_plan':'GET_PATH', 'start_plan':'FOLLOW_PATH', 'charge_mode':'CHARGE_MODE'})
    smach.StateMachine.add('CHARGE_MODE', ChargeMode(),
                           transitions={'finish_charge':'GET_PATH'})
    
  # For see state machine in diagram
  sis = smach_ros.IntrospectionServer('puma', sm, '/WAYPOINTS')
  sis.start()
  
  while not rospy.is_shutdown():
    outcome = sm.execute()
    sis.stop()
