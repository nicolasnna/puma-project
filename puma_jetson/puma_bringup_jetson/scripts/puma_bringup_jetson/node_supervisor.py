import time
import subprocess
import rospy
import threading
from std_msgs.msg import Empty
from puma_msgs.msg import Log

class NodeSupervisor:
  def __init__(self, node_name, node_command, topic_name="", type_topic=object, timeout=2, times_respawn=3):
    self.node_name = node_name
    self.topic_name = topic_name
    self.node_command = node_command
    self.timeout = timeout
    self.last_message_time = time.time()
    self.node_process = None
    self.running = True
    self.count_respawn = 0
    self.times_respawn = times_respawn
    self.last_time_reset = time.time()
    ''' Subscripción a tópicos'''
    if self.topic_name != "":
      rospy.Subscriber(self.topic_name, type_topic, self.message_callback) 
    rospy.Subscriber('/puma/supervisor/' + self.node_name + '/stop', Empty, self.stop_cb)
    rospy.Subscriber('/puma/supervisor/' + self.node_name + '/start', Empty, self.start_cb)
    ''' Publicación de tópicos'''
    self.log_pub = rospy.Publisher('/puma/logs/add_log', Log, queue_size=2)
    ''' Manejo de hilos'''
    self.start_node()
    self.thread_init()
    
  def thread_init(self): 
    self.thread = threading.Thread(target=self.supervise_node)
    self.thread.daemon = True
    self.thread.start()
  
  def is_node_running(self):
    try:
      output = subprocess.check_output(["rosnode", "list"])
      # rospy.loginfo(f"Nodos registrados: {output.decode('utf-8')}")
      return self.node_name in output.decode("utf-8")
    except subprocess.CalledProcessError:
      return False
      
  def send_log(self, msg, level):
    """Envía un mensaje de log al tópico de logs."""
    log = Log()
    log.level = level
    log.content = msg
    log.node = rospy.get_name() + "/" + self.node_name
    self.log_pub.publish(log)
    if level == 0:
      rospy.loginfo(msg)
    elif level == 1:
      rospy.logwarn(msg)
    elif level == 2:
      rospy.logerr(msg)

  def start_node(self):
    """Lanza el nodo como un proceso independiente."""
    if not self.is_node_running():
      rospy.loginfo(f"Iniciando nodo para {self.node_name}...")
      self.node_process = subprocess.Popen(self.node_command, shell=True)
      self.last_message_time = time.time()

  def stop_node(self):
    """Detiene el nodo si está ejecutándose."""
    if self.node_process and self.node_process.poll() is None:
      rospy.loginfo("Deteniendo nodo...")
      self.node_process.terminate()
      try:
        self.node_process.wait(timeout=3)
      except subprocess.TimeoutExpired:
        rospy.logwarn("El nodo no se detuvo a tiempo, forzando cierre...")
        self.node_process.kill()
        self.node_process.wait()
      rospy.sleep(1)
      # Verificar si el nodo sigue registrado en ROS
      if self.is_node_running():
        rospy.logwarn("El nodo sigue registrado en ROS, eliminando proceso...")
        subprocess.call(["rosnode", "kill", self.node_name])
      return True
    return False

  def stop_cb(self, msg):
    """Detiene el nodo si se recibe un mensaje de parada."""
    self.running = False
    rospy.sleep(0.5)
    self.stop_node()
    self.send_log(f"El nodo {self.node_name} ha sido detenido por mensaje.", 0)
      
  def start_cb(self, msg):
    """Inicia el nodo si se recibe un mensaje de inicio."""
    if not self.is_node_running():
      self.start_node()
      self.running = True
      self.count_respawn = 0
      self.thread_init()
      self.send_log(f"El nodo {self.node_name} ha sido iniciado por mensaje.", 0)

  def message_callback(self, msg):
    """Actualiza el tiempo del último mensaje recibido."""
    self.last_message_time = time.time()

  def supervise_node(self):
    """Hilo que monitorea el estado del nodo."""
    if self.topic_name != "":
      while self.running:
        elapsed_time = time.time() - self.last_message_time
        self.check_for_reset_count_respawn()
        if self.count_respawn >= self.times_respawn:
          self.send_log(f"Se ha alcanzado el límite de reinicios automaticos para el nodo {self.node_name}.", 2)
          self.running = False
          self.stop_node()
        elif elapsed_time > self.timeout:
          rospy.logwarn(f"No se detectaron mensajes en {self.topic_name} por {elapsed_time:.2f} segundos.")
          self.stop_node()
          time.sleep(3)  # Solo congela este hilo, no todo el programa
          self.start_node()
          self.count_respawn+=1
          self.last_time_reset = time.time()
        time.sleep(0.5)  # Intervalo de monitoreo
      self.stop_node()
    
  def check_for_reset_count_respawn(self):
    """Reinicia el contador de reinicios."""
    if time.time() - self.last_time_reset > 60:
      self.count_respawn = 0
      self.last_time_reset = time.time()
      
  def stop_supervisor(self):
    """Detiene la supervisión y el nodo."""
    self.running = False
    self.stop_node()
    rospy.loginfo(f"Supervisor para {self.topic_name} detenido.")