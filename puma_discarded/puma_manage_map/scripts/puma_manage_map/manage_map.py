#!/usr/bin/env python3
import rospy
import roslaunch
import rospkg
import time
from puma_manage_map_msgs.msg import ManageCmd
from std_msgs.msg import Empty

class ManageMap():
  def __init__(self):
    ns = '/manage_map'
    self.ns_robot = '/puma/map'
    folder = rospy.get_param(ns+'/folder', 'test')
    self.path_map = rospkg.RosPack().get_path('puma_manage_map')+"/maps/"+folder
    map_origin = self.get_map_name(0,0)
    # Args node map_server
    self.package = 'map_server'
    self.name = 'map_server'
    self.type = 'map_server'
    args = self.path_map + '/' + map_origin
    node_map = roslaunch.core.Node(self.package,self.type,self.name, args=args)
    
    # init roslaunch
    self.launch = roslaunch.scriptapi.ROSLaunch()
    self.launch.start()
    self.map_process = self.launch.launch(node_map) 
    
    # Subscriber to cmd
    rospy.Subscriber(self.ns_robot+'/change_map', ManageCmd, self.change_map)
    # Publish ready charge map
    self.ready_map_publish = rospy.Publisher(self.ns_robot+'/map_ready', Empty, queue_size=1)
    
  def get_map_name(self, x, y):
    """ Return file name with nomenclature in cartesian coords """
    return 'map-c'+str(x)+'_'+str(y)+'.yaml'
  
  def change_map(self, coords):
    """ Callback when published cmd """
    file_name = self.get_map_name(coords.coord_x, coords.coord_y)
    rospy.loginfo(' Cambiando a mapa %s', file_name)
    # Reinit node with new args
    new_args = self.path_map + '/' + file_name
    node_map = roslaunch.core.Node(self.package,self.type,self.name, args=new_args)
    self.map_process.stop()
    try:
      self.map_process = self.launch.launch(node_map)
      time.sleep(0.5)
      if self.map_process.is_alive():
        self.ready_map_publish.publish(Empty())
        rospy.loginfo('Se ha relanzado el nodo map correctamente.')
    except Exception as e:
      rospy.logerr('Se ha producido un error al relanzar el nodo: %s', e)