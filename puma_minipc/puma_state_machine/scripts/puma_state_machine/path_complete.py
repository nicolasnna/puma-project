#!/usr/bin/env python3
import rospy
import smach
from std_msgs.msg import Empty

class PathComplete(smach.State):
  """ State when is completed path planned """
  def __init__(self):
    smach.State.__init__(self, outcomes=['start_plan', 'select_plan','charge_mode'], input_keys=['waypoints'], output_keys=['waypoints'])
    
  def start_subscriber(self):
    ns_topic = rospy.get_param('~ns_topic','')
    self.reset_sub = rospy.Subscriber(ns_topic+"/plan_ready", Empty, self.start_plan_callback)
    self.select_path_sub = rospy.Subscriber(ns_topic+"/plan_reset", Empty, self.select_plan_callback)
    self.charge_mode_sub = rospy.Subscriber(ns_topic+"/run_charge_mode", Empty, self.charge_mode_callback)
    
  def end_subscriber(self):
    self.reset_sub.unregister()
    self.select_path_sub.unregister()
    self.charge_mode_sub.unregister()
    
  def start_plan_callback(self, msg):
    rospy.loginfo("-> Recibido comando para volver a ejecutar el plan.")
    self.start_plan = True
    
  def select_plan_callback(self, msg):
    rospy.loginfo("-> Recibido comando para entrar en la seleccion de waypoints.")
    self.select_plan = True
    
  def charge_mode_callback(self, msg):
    rospy.loginfo("-> Recibido comando para entrar en modo carga.")
    self.charge_mode = True
    
  def execute(self, ud):
    rospy.loginfo('--------------------------------')
    rospy.loginfo('----- Estado Path Complete -----')
    rospy.loginfo('--------------------------------')
    self.start_plan = self.select_plan = self.charge_mode = False
    ns_topic = rospy.get_param('~ns_topic','')
    self.start_subscriber()
    
    rospy.loginfo("Para volver a ejecutar enviar std_msgs/Empty en %s/plan_ready", ns_topic)
    rospy.loginfo("Para entrar a seleccion waypoints std_msgs/Empty en %s/plan_reset", ns_topic)
    rospy.loginfo("Para entrar a modo carga std_msgs/Empty en %s/run_charge_mode", ns_topic)

    ''' Bucle a la espera de respuesta del usuario '''
    try:
      while not rospy.is_shutdown() and not self.select_plan and not self.start_plan and not self.charge_mode:
        rospy.Rate(10).sleep()
    except rospy.ROSInterruptException:
      rospy.logwarn("-> Cerrando puma_state_machine -- PATH_COMPLETE.")
      
    self.end_subscriber()
    
    if self.select_plan:
      return 'select_plan'
    if self.start_plan:
      return 'start_plan'
    if self.charge_mode:
      return 'charge_mode'