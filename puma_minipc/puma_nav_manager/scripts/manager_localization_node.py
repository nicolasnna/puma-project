#!/usr/bin/env python3  
import rospy
from robot_localization.srv import SetPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import actionlib
from puma_nav_manager.msg import LocalizationManagerAction, LocalizationManagerFeedback, LocalizationManagerResult, LocalizationManagerGoal
import math

def set_pose_callback(msg):
  try:
    rospy.wait_for_service('/set_pose', timeout=5)
    set_pose = rospy.ServiceProxy('/set_pose', SetPose)
    set_pose(msg)
  except rospy.ServiceException as e:
    rospy.logerr(f"Service call failed: {e}")
    
def odom_cb(msg):
  global odom
  odom = msg
  
def odom_map_cb(msg):
  global odom_map
  odom_map = msg

def use_service_set_pose(newPose, ns_service):
  try:
    rospy.wait_for_service(f'{ns_service}/set_pose', timeout=5)
    set_pose = rospy.ServiceProxy(f'{ns_service}/set_pose', SetPose)
    set_pose(newPose)
    set_pose.close()
  except rospy.ServiceException as e:
    rospy.logerr(f"Service call failed: {e}")

def change_angle_degree_cb(msg):
  global odom, odom_map
  newPose = PoseWithCovarianceStamped()
  newPose.pose.pose.position.x = odom.pose.pose.position.x
  newPose.pose.pose.position.y = odom.pose.pose.position.y
  newPose.pose.pose.position.z = odom.pose.pose.position.z
  newPose.pose.pose.orientation.x = 0
  newPose.pose.pose.orientation.y = 0
  newPose.pose.pose.orientation.z = math.sin(math.radians(-msg.data)/2)
  newPose.pose.pose.orientation.w = math.cos(math.radians(-msg.data)/2)
  newPose.header.stamp = rospy.Time.now()
  newPose.header.frame_id = odom.header.frame_id
  
  use_service_set_pose(newPose, 'puma_ekf_odom')
  
  if odom_map:
    newPose.header.frame_id = odom_map.header.frame_id
    newPose.pose.pose.position.x = odom_map.pose.pose.position.x
    newPose.pose.pose.position.y = odom_map.pose.pose.position.y
    newPose.pose.pose.position.z = odom_map.pose.pose.position.z
    use_service_set_pose(newPose, 'puma_ekf_map')
  
def reset_position_cb():
  global odom, odom_map
  newPoseOdom = odom
  newPoseOdom.pose.pose.position.x = 0
  newPoseOdom.pose.pose.position.y = 0
  newPoseOdom.pose.pose.position.z = 0
  use_service_set_pose(newPoseOdom, 'puma_ekf_odom')
  if odom_map:
    newPoseMap = odom_map
    newPoseMap.pose.pose.position.x = 0
    newPoseMap.pose.pose.position.y = 0
    newPoseMap.pose.pose.position.z = 0
    use_service_set_pose(newPoseMap, 'puma_ekf_map')
  
def server_cb(goal: LocalizationManagerGoal):
  global srv
  result = LocalizationManagerResult()
  
  try: 
    if goal.action == LocalizationManagerGoal.ACTION_CHANGE_ANGLE:
      change_angle_degree_cb(goal.angle_deg)
      result.success = True
      result.message = 'Ángulo cambiado con éxito'
    elif goal.action == LocalizationManagerGoal.ACTION_RESET_POSITION:
      reset_position_cb()
      result.success = True
      result.message = 'Posición reiniciada con éxito'
    else:
      result.success = False
      result.message = 'Acción no válida'
    
    srv.set_succeeded(result)
  
  except Exception as e:
    rospy.logwarn(f"Error al ejecutar acción: {e}")
    result.success = False
    result.message = 'Error al ejecutar acción'
    srv.set_succeeded(result)
  
  
def main():
  rospy.init_node('manage_localization_ekf')
  
  global odom, odom_map, srv
  odom = odom_map = None
  
  rospy.Subscriber('/puma/localization/set_new_pose', PoseWithCovarianceStamped, set_pose_callback)
  rospy.Subscriber('/puma/localization/change_angle_degree', Float64, change_angle_degree_cb)
  rospy.Subscriber('/puma/localization/ekf_odometry', Odometry, odom_cb)
  rospy.Subscriber('/puma/localization/filtered_map', Odometry, odom_map_cb)
  
  rospy.loginfo("Nodo manager_localization iniciado")
  
  
  srv = actionlib.SimpleActionServer('/puma/localization/manager', LocalizationManagerAction, server_cb, False)
  srv.start()
  
  rospy.spin()

if __name__ == '__main__':
  main()