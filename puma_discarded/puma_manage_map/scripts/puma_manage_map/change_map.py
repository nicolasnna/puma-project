#!/usr/bin/env python3
import rospy
from puma_manage_map_msgs.msg import ManageCmd
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry

class ChangeMap():
  def __init__(self):
    ns = '/change_map/'
    map_publisher_topic = rospy.get_param(ns+'topic', '/puma/map/change_map')
    self.map_publisher = rospy.Publisher(map_publisher_topic, ManageCmd, queue_size=1)
    rospy.Subscriber('odometry/filtered', Odometry, self.get_position_robot)
    rospy.Subscriber('puma/waypoints/path_planned', PoseArray, self.get_goal)
    
    self.res = rospy.get_param(ns+'resolution', 0.1)
    self.width = rospy.get_param(ns+'width', 300) * self.res
    self.height = rospy.get_param(ns+'height', 300) * self.res
    self.over = rospy.get_param(ns+'over', 30) * self.res
    
    self.coords_x = self.coords_y = 0
    self.robot_x = self.robot_y = 0
    self.goal_x = self.goal_y = 0
    
    self.get_limits()
    
  def get_goal(self, plan_array):
    """ Get position of next goal """
    if plan_array.poses:
      self.goal_x = plan_array.poses[0].position.x
      self.goal_y = plan_array.poses[0].position.y
    
  def get_position_robot(self, msg_odom):
    """ Get pos x and y of robot """
    self.robot_x = msg_odom.pose.pose.position.x
    self.robot_y = msg_odom.pose.pose.position.y
    
  def get_limits(self):
    """ calculate limits """
    self.origin_x = (self.width) * (self.coords_x )
    self.origin_y = (self.height) * (self.coords_y )
    
    # Define limit
    self.x_left_limit = -self.width/2 + self.origin_x
    self.x_right_limit = self.width/2 + self.origin_x
    
    self.y_down_limit = -self.height/2 + self.origin_y
    self.y_up_limit = self.height/2 + self.origin_y 
    
  def check_quadrants(self):
    """ Check and change map if is necessary """
    self.get_limits()
    is_enable_x_right = self.robot_x >= self.x_right_limit and self.goal_x >= self.x_right_limit
    is_enable_x_left = self.robot_x <= self.x_left_limit and self.goal_x <= self.x_left_limit
    is_enable_y_up = self.robot_y >= self.y_up_limit and self.goal_y >= self.y_up_limit 
    is_enable_y_down = self.robot_y <= self.y_down_limit and self.goal_y <= self.y_down_limit
    
    is_need_change_map = False
    
    if is_enable_x_right:
      self.coords_x += 1
      is_need_change_map = True
    elif is_enable_x_left:
      self.coords_x -= 1
      is_need_change_map = True
    
    if is_enable_y_up:
      self.coords_y += 1
      is_need_change_map = True
    elif is_enable_y_down:
      self.coords_y -= 1
      is_need_change_map = True
      
    if is_need_change_map:
      rospy.loginfo('Realizando cambio de cuadrante: %d, %d', self.coords_x, self.coords_y)
      self.change_map(self.coords_x, self.coords_y)
    
  def change_map(self, x, y):
    """ Send cmd to manage map for change current map """
    map_cmd = ManageCmd()
    map_cmd.coord_x = x
    map_cmd.coord_y = y
    
    self.map_publisher.publish(map_cmd)