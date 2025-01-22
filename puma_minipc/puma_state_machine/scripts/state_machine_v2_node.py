#!/usr/bin/env python3
import rospy
from puma_state_machine.plan_configuration import PlanConfiguration
from puma_state_machine.run_plan import RunPlan
import smach
import smach_ros

if __name__ == "__main__":
  rospy.init_node('puma_state_machine')
  sm = smach.StateMachine(outcomes=['success'])
  
  with sm:
    smach.StateMachine.add('PLAN_CONFIGURATION',PlanConfiguration(),
                          transitions={'run_plan':'RUN_PLAN', 'run_plan_schedule':'RUN_PLAN'})
    smach.StateMachine.add('RUN_PLAN',RunPlan(),
                          transitions={'success':'PLAN_CONFIGURATION', 'plan_configuration':'PLAN_CONFIGURATION'})
    
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