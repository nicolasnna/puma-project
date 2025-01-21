import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String, Header
from puma_msgs.msg import WaypointNav, Waypoint
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import json
import rospkg

class FilesManager:
  def __init__(self, ns, ns_waypoints, ns_plan):
    self._plan : Path = Path()
    self.local_files_path = rospkg.RosPack().get_path('puma_nav_manager') + "/nav_path"
    self._ns_waypoints = ns_waypoints
    self._ns_plan = ns_plan
    self.init_publishers(ns, ns_waypoints, ns_plan)
    self.start_subscriber(ns)
    
  def init_publishers(self, ns, ns_waypoints, ns_plan):
    self.plan_select_pub = rospy.Publisher(ns+'/plan_selected', Path, queue_size=5)
    self.waypoint_set_pub = rospy.Publisher(ns_waypoints+'/set', WaypointNav, queue_size=5)
    self.plan_add_pub = rospy.Publisher(ns_plan+'/add', Path, queue_size=5)
  
  def start_subscriber(self, ns):
    self.set_plan_sub = rospy.Subscriber(ns+'/set_plan', Path, self.set_plan_callback)
    self.clear_plan_sub = rospy.Subscriber(ns+'/clear_plan', Empty, self.clear_plan_callback)
    self.export_plan_local_sub = rospy.Subscriber(ns+'/export_plan_local', String, self.export_plan_local_callback)
    self.import_plan_local_sub = rospy.Subscriber(ns+'/import_plan_local', String, self.import_plan_local_callback)
    
  def import_plan_local_callback(self, file_name):
    path_json = self.local_files_path + '/' + file_name.data + '.json'
    try:
      waypoints, plan = self.import_plan_from_json(path_json)
      self.waypoint_set_pub.publish(waypoints)
      self.plan_add_pub.publish(plan)
      rospy.sleep(0.1)
      rospy.loginfo(f'Plan importado con éxito con el nombre {file_name.data}.json')
    except Exception as e:
      rospy.logerr(f'Error al importar plan: {e}')
    
  def export_plan_local_callback(self, file_name):
    if self._plan.poses:
      try:
        waypoint_list = rospy.wait_for_message(self._ns_waypoints + '/waypoints_info', WaypointNav, timeout=5)
        path_json = self.local_files_path + '/' + file_name.data + '.json'
        self.export_plan_to_json(path_json, waypoint_list)
        rospy.loginfo(f'Plan exportado con éxito con el nombre {file_name.data}.json')
      except Exception as e:
        rospy.logerr(f'Error al exportar plan: {e}')
        
    
  def set_plan_callback(self, plan):
    self._plan = plan
  
  def clear_plan_callback(self, empty):
    del self._plan
    self._plan = Path()
    
  def publish_status(self):
    self.plan_select_pub.publish(self._plan)
    
  def export_plan_to_json(self, file_path: String, waypoints: WaypointNav):
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
    
    plan_info = {
      'header': {
        'frame_id': self._plan.header.frame_id,
        'stamp': self._plan.header.stamp.to_nsec(),
        'seq': self._plan.header.seq
      },
      'poses': [
        {
          'header': {
            'frame_id': pose.header.frame_id,
            'stamp': pose.header.stamp.to_nsec(),
            'seq': pose.header.seq
          },
          'pose': {
            'position': {
              'x': pose.pose.position.x,
              'y': pose.pose.position.y,
              'z': pose.pose.position.z
            },
            'orientation': {
              'x': pose.pose.orientation.x,
              'y': pose.pose.orientation.y,
              'z': pose.pose.orientation.z,
              'w': pose.pose.orientation.w
            }
          }
        } for pose in self._plan.poses
      ]
    }
    
    json_export = {
      'waypoints': waypoints_info,
      'plan': plan_info
    }
    
    with open(file_path, 'w') as file:
      json.dump(json_export, file, indent=2, sort_keys=True)
      
  def import_plan_from_json(self, file_path):
    with open(file_path, 'r') as file:
      json_import = json.load(file)
      
    waypoints_info = json_import['waypoints']
    plan_info = json_import['plan']
    
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
    
    plan = Path()
    plan.header = Header(
      frame_id=plan_info['header']['frame_id'],
      stamp=rospy.Time.from_sec(plan_info['header']['stamp'] / 1e9),
      seq=plan_info['header']['seq']
    )
    plan.poses = [
      PoseStamped(
        header=Header(
          frame_id=pose['header']['frame_id'],
          stamp=rospy.Time.from_sec(pose['header']['stamp'] / 1e9),
          seq=pose['header']['seq']
        ),
        pose=Pose(
          position=Point(
            x=pose['pose']['position']['x'],
            y=pose['pose']['position']['y'],
            z=pose['pose']['position']['z']
          ),
          orientation=Quaternion(
            x=pose['pose']['orientation']['x'],
            y=pose['pose']['orientation']['y'],
            z=pose['pose']['orientation']['z'],
            w=pose['pose']['orientation']['w']
          )
        )
      ) for pose in plan_info['poses']
    ]
    
    return waypoints, plan