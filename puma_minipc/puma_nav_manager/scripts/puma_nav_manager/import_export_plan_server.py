#!/usr/bin/env python
import rospy
import actionlib
import json
import rospkg
from puma_msgs.msg import WaypointNav, Waypoint
from std_msgs.msg import Header
from puma_nav_manager.msg import ImportExportPlanAction, ImportExportPlanFeedback, ImportExportPlanResult
import os

class ImportExportPlanServer:
  def __init__(self, ns):
    ''' Manejador para importar y exportar waypoints '''
    self._server = actionlib.SimpleActionServer(ns + '/files_manager', ImportExportPlanAction, self.execute_cb, False)
    self._server.start()
    rospy.loginfo('Servidor files manager iniciado')
    
    self._ns = ns
    self.local_files_path = rospkg.RosPack().get_path('puma_nav_manager') + "/nav_path"
    
  def export_plan_to_json(self, file_path, waypoints: WaypointNav):
    waypoints_info = {
      'header': {
        'frame_id': waypoints.header.frame_id,
        'stamp': waypoints.header.stamp.to_nsec(),
        'seq': waypoints.header.seq
      },
      'waypoints': [
        {
          'x': waypoint.x,
          'y': waypoint.y,
          'yaw': waypoint.yaw,
          'latitude': waypoint.latitude,
          'longitude': waypoint.longitude
        } for waypoint in waypoints.waypoints
      ]
    }
    
    json_export = {
      'waypoints': waypoints_info,
    }
    
    with open(file_path, 'w') as file:
      json.dump(json_export, file, indent=2, sort_keys=True)

  def execute_export(self, file_name): 
    feedback = ImportExportPlanFeedback()
    result = ImportExportPlanResult()
    
    rospy.loginfo('Exportando plan para archivo %s', file_name)
    
    path_json = self.local_files_path + "/" + file_name + ".json"
    
    try: 
      waypoint_list = rospy.wait_for_message(self._ns + "/waypoints_list", WaypointNav, timeout=5)
      
      self.export_plan_to_json(path_json, waypoint_list)
      
      if os.path.isfile(path_json):
        result.success = True
        result.message = "Plan exportado exitosamente"
      else:
        result.success = False
        result.message = "Error al exportar plan. No se ha logrado crear el archivo"
      
      rospy.loginfo(result.message)
      
    except Exception as e:
      result.success = False
      result.message = f"Error al exportar plan: {e}"
      rospy.logerr(result.message)
    
    return result
    
  def import_plan_from_json(self, file_path):
    with open(file_path, 'r') as file:
      json_import = json.load(file)
      
    waypoints_info = json_import['waypoints']
        
    waypoints = WaypointNav()
    waypoints.header = Header(
      frame_id=waypoints_info['header']['frame_id'],
      stamp=rospy.Time.from_sec(waypoints_info['header']['stamp'] / 1e9),
      seq=waypoints_info['header']['seq']
    )
    waypoints.waypoints = [
      Waypoint(
        x=wp['x'],
        y=wp['y'],
        yaw=wp['yaw'],
        latitude=wp['latitude'],
        longitude=wp['longitude']
      ) for wp in waypoints_info['waypoints']
    ]
    
    return waypoints
  
  def execute_import(self, file_name):
    feedback = ImportExportPlanFeedback()
    result = ImportExportPlanResult()
    
    rospy.loginfo('Importando plan desde archivo %s', file_name)
    
    path_json = self.local_files_path + "/" + file_name + ".json"
    
    try: 
      waypoints = self.import_plan_from_json(path_json)
      
      wp_test = rospy.wait_for_message(self._ns + "/waypoints_list", WaypointNav, timeout=5)
      
      waypoints_pub = rospy.Publisher(self._ns + "/set_waypoints", WaypointNav, queue_size=10)
      
      i = 0
      while i < 10:
        waypoints_pub.publish(waypoints)
        
        rospy.sleep(0.3)
        wp = rospy.wait_for_message(self._ns + "/waypoints_list", WaypointNav, timeout=5)
        if wp.waypoints == waypoints.waypoints:
          break
        i+=1
      
      if wp.waypoints == waypoints.waypoints:
        result.success = True
        result.message = "Plan importado exitosamente"
      else:
        result.success = False
        result.message = "Error al importar plan. Waypoints no coinciden"
        
      rospy.loginfo(result.message)
      
    except Exception as e:
      result.success = False
      result.message = f"Error al importar plan: {e}"
      rospy.logerr(result.message)
      
    return result
    
  def execute_cb(self, goal):
    if not goal.file_name:
      rospy.logerr('Nombre de archivo no proporcionado')
      self._server.set_aborted()
      return

    if goal.action == "import": 
      result = self.execute_import(goal.file_name)
    elif goal.action == "export":
      result = self.execute_export(goal.file_name)
    else:
      result = ImportExportPlanResult()
      result.success = False
      result.message = "Acción no válida"
      
    self._server.set_succeeded(result)