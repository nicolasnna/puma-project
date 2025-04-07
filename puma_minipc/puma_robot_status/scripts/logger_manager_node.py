import rospy
import rospkg
from puma_msgs.msg import Log, LogArray
from std_msgs.msg import Empty
from datetime import datetime
from puma_robot_status.msg import LoggerManagerAction, LoggerManagerGoal, LoggerManagerResult
from actionlib import SimpleActionServer
from pathlib import Path

class writeToFile:
  def __init__(self):
    is_simulated = rospy.get_param('~is_simulation', False)
    
    project_root = Path(__file__).resolve().parent.parent  # Sube a puma_robot_status
    tmp_dir = project_root / "tmp" / "logs" / "simulation"
    tmp_dir.mkdir(parents=True, exist_ok=True)
        
    directory = rospkg.RosPack().get_path('puma_robot_status') + '/tmp/logs/'
    if is_simulated:
      self.filename = directory + "simulation/" + "log simulation -" + datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '.txt'
    else:
      self.filename = directory + "log sesion -" + datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '.txt'
    
  def write(self, data):
    with open(self.filename, 'a') as file:
      file.write(data + '\n')

class LogManager:
  def __init__(self):
    self.logs_data = LogArray()
    rospy.Subscriber('/puma/logs/add_log', Log, self.log_received)
    rospy.Subscriber('/puma/logs/clean_logs', Empty, self.clean_logs)
    self.log_publisher = rospy.Publisher('/puma/logs/logs', LogArray, queue_size=2)
    self._srv = SimpleActionServer('/puma/logs', LoggerManagerAction, self.execute_srv, False)
    self._srv.start()
    self.writeToFile = writeToFile()

  def log_received(self, log: Log):
    # Añade un log con la fecha actual
    log.date_text = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    self.logs_data.logs.append(log)
    text = f"{log.date_text} ; {log.level} ; {log.node} ; {log.content}"
    try:
      self.writeToFile.write(text)
    except Exception as e:
      rospy.logwarn(f"Error al escribir el log en el archivo: {e}")
  
  def clean_logs(self, _):
    # Limpia los logs
    self.logs_data = LogArray()

  def publish_logs(self):
    # Publica los logs actuales
    self.log_publisher.publish(self.logs_data)
    
  def execute_srv(self, goal: LoggerManagerGoal):
    result = LoggerManagerResult()
    
    if goal.action == LoggerManagerGoal.GET_LOG_AND_CLEAN:
      result.log_array = self.logs_data
      result.success = True
      result.message = "Logs obtenidos correctamente"
      self.clean_logs(Empty())
    else:
      result.success = False
      result.message = "Acción no válida"
    
    self._srv.set_succeeded(result)

if __name__ == "__main__":
  rospy.init_node('manager_logger')
  
  log_manager = LogManager()
  
  rate = rospy.Rate(2)
  try: 
    while not rospy.is_shutdown():
      log_manager.publish_logs()
      rate.sleep()
      
  except Exception as e: 
    rospy.logwarn(f"El nodo {rospy.get_name()} se ha cerrado debido al error: {e}")