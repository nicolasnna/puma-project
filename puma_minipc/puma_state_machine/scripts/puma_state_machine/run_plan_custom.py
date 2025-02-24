import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction
from puma_state_machine.utils import (check_and_get_waypoints,
                                      check_mode_control_navegacion,
                                      get_goal_from_waypoint)
from std_msgs.msg import String
from std_srvs.srv import Empty as EmptySrv
from .run_plan import RunPlan
import smach
from puma_state_machine.msg import StateMachineAction

class RunPlanCustom(RunPlan):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success', 'plan_configuration'], input_keys=['plan_configuration_info'])
    
    self.limit_time_status_mb = 0.5
    self.publisher()
    ns_topic = rospy.get_param('~ns_topic', 'state_machine')
    self._srv = actionlib.SimpleActionServer(ns_topic + "/run_plan_custom", StateMachineAction, self.execute_srv_cb, False)
    self._srv.start()

  
  def execute(self, ud):
    rospy.loginfo('----- Estado ejecución de plan personalizado -----')
    self.send_log("Iniciando en el estado de ejecución de plan de navegación.", 0)
    ''' Variables '''
    self.is_aborted = False
    self.status_move_base = 0
    ''' Definir modo navegacion en el control puma '''
    self.mode_selector_pub.publish(String(data='navegacion'))
    rospy.sleep(0.3)
    
    ''' Comprobar si se ha cambiado al modo de navegación '''
    if not check_mode_control_navegacion('navegacion', self.send_log):
      return 'plan_configuration'
    
    ''' Abrir cliente move_base '''
    self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.client.wait_for_server()
    rospy.loginfo("Conectado con move_base...")
    ''' Iniciar suscripciones '''
    self.start_subscriber()
    
    
    ''' Limpiar costmaps '''
    rospy.wait_for_service('/move_base/clear_costmaps')
    service_clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', EmptySrv)

    ''' Revisar waypoints subidos '''
    exist_waypoints, waypoints_msg = check_and_get_waypoints(self.send_log)
    if not exist_waypoints:
      return 'plan_configuration'

    ''' Ejecutar los planes '''
    try:
      for loops in range(ud.plan_configuration_info['repeat']):
        self.reset_progress_wp()
        
        for waypoint in waypoints_msg.waypoints:
          try: 
            service_clear_costmaps()
          except rospy.ServiceException as e:
            rospy.logerr(f"Error al limpiar los costmaps: {e}")
          rospy.sleep(0.2)
          
          goal = get_goal_from_waypoint(waypoint)
          is_complete = False
          self.send_log("Ejecutando el plan de navegación...", 0)
          self.client.send_goal(goal)
          rospy.sleep(0.2)
          while not is_complete:
            if self.is_aborted:
              break
            self.check_move_base_status()
            is_complete = self.client.wait_for_result(rospy.Duration(1))
          if self.is_aborted:
            break
          rospy.sleep(0.3)
          if self.status_move_base == 4:
            self.send_log(f"Waypoint ({round(waypoint.x,2), round(waypoint.y,2)}) ha sido abortado por el sistema de navegación. Revisar estado del robot. Volviendo a configuración de plan.", 1)
            self.is_aborted = True
            break
          if is_complete:
            self.send_log(f"Waypoint ({round(waypoint.x,2), round(waypoint.y,2)}) completado.", 0)
            self.update_progress_wp(waypoint)
        
        if self.is_aborted:
          break
        rospy.sleep(rospy.Duration(ud.plan_configuration_info['minutes_between_repeats']*60))
      self.send_log(f"Plan de navegación completado con {loops+1} vueltas.", 0)
      
    except Exception as e:
      self.client.cancel_all_goals()
      rospy.sleep(0.1)
      self.send_log(f"No se ha podido efectuar el plan por el error: {e}. Volviendo a configuración de planes", 1)
      self.is_aborted = True
    
    self.end_subscriber()
    self.mode_selector_pub.publish(String(data='idle'))
    
    ''' Comprobar si se ha completado el plan '''
    if self.is_aborted:
      return 'plan_configuration'
    
    return 'success'