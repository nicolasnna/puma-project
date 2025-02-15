#!/usr/bin/env python3
import rospy
from puma_state_machine.path_select import PathSelect
from puma_state_machine.path_follow import PathFollow
from puma_state_machine.path_complete import PathComplete
from puma_state_machine.charge_mode import ChargeMode
import smach
import smach_ros

if __name__ == "__main__":
  rospy.init_node('puma_state_machine')
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
  sis = smach_ros.IntrospectionServer('puma', sm, '/state_machine')
  sis.start()
  
  try:
    outcome = sm.execute()
  except:
    rospy.logwarn("Interrupción detectada. Cerrando puma_state_machine.")
  finally:
    sis.stop()  # Ensure introspection server is stopped if the node is interrupted
    rospy.logwarn("Nodo puma_state_machine cerrado correctamente.")