#!/usr/bin/env python
import time
import threading
import rospy
import roslaunch
from std_msgs.msg import Empty  # Ejemplo: para comandos de start/stop
from puma_msgs.msg import Log

class NodeSupervisorLaunch:
  def __init__(self, node_name, launch_file, monitor_topic="",  type_topic=object, timeout=2, times_respawn=3):
    """
    :param node_name: Nombre del nodo (tal como se registra en ROS)
    :param launch_file: Ruta al archivo .launch que arranca el nodo
    :param monitor_topic: Tópico que se usará para determinar si el nodo está activo.
                          Si se deja en cadena vacía, la supervisión basada en tópico no se usará.
    :param timeout: Tiempo en segundos sin recibir mensajes para reiniciar el nodo.
    :param times_respawn: Número máximo de reinicios automáticos permitidos.
    """
    self.node_name = node_name
    self.launch_file = launch_file
    self.monitor_topic = monitor_topic
    self.timeout = timeout
    self.times_respawn = times_respawn
    self.count_respawn = 0
    self.last_message_time = time.time()
    self.last_time_reset = time.time()
    self.running = True
    self.stop_process = False
    self.node_parent = None  # Objeto ROSLaunchParent
    # Si se especifica un tópico para monitoreo, nos suscribimos
    if self.monitor_topic != "":
        rospy.Subscriber(self.monitor_topic, type_topic, self.message_callback)
    # Comandos para detener o iniciar el nodo a través de tópicos supervisor (opcional)
    rospy.Subscriber('/supervisor/' + self.node_name + '/stop', Empty, self.stop_cb)
    rospy.Subscriber('/supervisor/' + self.node_name + '/start', Empty, self.start_cb)
    
    ''' Publicación de tópicos'''
    self.log_pub = rospy.Publisher('/puma/logs/add_log', Log, queue_size=2)
    
    # Inicia el nodo y la supervisión
    self.start_node()
    self.thread = threading.Thread(target=self.supervise_node)
    self.thread.daemon = True
    self.thread.start()
      
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
    """Inicia el nodo usando la API de roslaunch."""
    # Genera un UUID para el launch y configura el logging
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    self.send_log("Lanzando nodo '{}' por medio del supervisor.".format(self.node_name), 0)
    self.node_parent = roslaunch.parent.ROSLaunchParent(uuid, [self.launch_file])
    self.node_parent.start()
    self.last_message_time = time.time()
    # Se espera un poco para que el nodo se estabilice
    rospy.sleep(4)
  
  def stop_node(self):
    """Detiene el nodo utilizando la API de roslaunch."""
    if self.node_parent is not None:
      self.send_log("Deteniendo nodo '{}'.".format(self.node_name), 1)
      self.node_parent.shutdown()
      self.node_parent = None
      rospy.sleep(1)
  
  def message_callback(self, msg):
    """Actualiza el tiempo del último mensaje recibido del tópico de monitoreo."""
    self.last_message_time = time.time()
  
  def stop_cb(self, msg):
    """Callback para detener el nodo manualmente."""
    self.send_log("Recibido comando para detener el nodo '{}'".format(self.node_name), 0)
    self.running = False
    self.stop_process = True
    self.stop_node()
    self.thread.join()
  
  def start_cb(self, msg):
    """Callback para iniciar el nodo manualmente."""
    if not self.running:
      self.send_log("Recibido comando para iniciar el nodo '{}'".format(self.node_name), 0)
      self.start_node()
      self.running = True
      self.count_respawn = 0
      self.stop_process = False
      self.thread = threading.Thread(target=self.supervise_node)
      self.thread.daemon = True
      self.thread.start()
    else:
      self.send_log("Recibido comando para iniciar el nodo '{}', pero el nodo ya esta iniciado. ".format(self.node_name), 0)

  
  def check_for_reset_count_respawn(self):
    """Reinicia el contador de reinicios si ha pasado un minuto sin reinicios."""
    if self.stop_process:
        self.count_respawn = self.times_respawn
    elif time.time() - self.last_time_reset > 60:
        self.count_respawn = 0
        self.last_time_reset = time.time()
  
  def supervise_node(self):
    """Hilo que monitorea el estado del nodo y lo reinicia si no se detecta actividad."""
    # Si no se especificó un tópico de monitoreo, no se puede supervisar basado en mensajes
    if self.monitor_topic != "":
        while self.running:
            elapsed_time = time.time() - self.last_message_time
            self.check_for_reset_count_respawn()
            if self.count_respawn >= self.times_respawn:
                self.send_log("Se alcanzó el límite de reinicios automáticos para el nodo '{}'.".format(self.node_name), 2)
                self.running = False
                self.stop_node()
                break
            elif elapsed_time > self.timeout:
                self.send_log("No se detectaron mensajes en el tópico '{}' por {:.2f} segundos. Reiniciando nodo '{}'.".format(
                    self.monitor_topic, elapsed_time, self.node_name), 1)
                self.stop_node()
                rospy.sleep(3)  # Espera en este hilo sin bloquear el resto del programa
                self.start_node()
                self.count_respawn += 1
                self.last_time_reset = time.time()
            rospy.sleep(0.5)
        # self.stop_node()
    else:
        self.send_log("No se ha definido un tópico de monitoreo para el nodo '{}'. Supervisión automática deshabilitada.".format(self.node_name), 1)
  
  def stop_supervisor(self):
    """Detiene la supervisión y apaga el nodo."""
    self.running = False
    self.stop_node()
    self.send_log("Supervisor para el nodo '{}' detenido.".format(self.node_name), 0)

