import rospy
from puma_msgs.msg import Log, LogArray
from std_msgs.msg import Empty
from datetime import datetime

class LogManager:
  def __init__(self):
    self.logs_data = LogArray()
    rospy.Subscriber('/puma/logs/add_log', Log, self.log_received)
    rospy.Subscriber('/puma/logs/clean_logs', Empty, self.clean_logs)
    self.log_publisher = rospy.Publisher('/puma/logs/logs', LogArray, queue_size=2)

  def log_received(self, log):
    # Añade un log con la fecha actual
    log.date_text = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    self.logs_data.logs.append(log)

  def clean_logs(self, _):
    # Limpia los logs
    self.logs_data = LogArray()

  def publish_logs(self):
    # Publica los logs actuales
    self.log_publisher.publish(self.logs_data)

if __name__ == "__main__":
  rospy.init_node('send_logs_web_node')
  global logsData
  logsData = LogArray()
  
  log_manager = LogManager()
  
  rate = rospy.Rate(2)
  try: 
    while not rospy.is_shutdown():
      log_manager.publish_logs()
      rate.sleep()
      
  except Exception as e: 
    rospy.logwarn(f"El nodo {rospy.get_name()} se ha cerrado debido al error: {e}")