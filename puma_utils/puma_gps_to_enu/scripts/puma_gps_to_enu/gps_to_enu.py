#!/usr/bin/env pyhton3
import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty

class GpsToEnu():
  def __init__(self):
    # Gets params from rosparams
    ns = '/gps_to_enu'
    # topic_gps = rospy.get_param('~gps_to_enu/topic_gps', 'gps/fix')
    topic_gps = rospy.get_param(ns+'/topic_gps', 'gps/fix')
    topic_publish = rospy.get_param(ns+'/topic_publish', 'puma/sensors/gps/enu')
    topic_reset = rospy.get_param(ns+'/topic_reset', 'puma/sensors/gps/reset')
    latitude_ref = rospy.get_param(ns+'/latitude_ref', -33.42196309675523)
    longitude_ref = rospy.get_param(ns+'/longitude_ref', -70.5818018059048)
    altitude_ref = rospy.get_param(ns+'/altitude_ref', 674)
    self.frame_id = rospy.get_param(ns+'/frame_id', 'map')
    self.apply_offset_yaw = rospy.get_param(ns+'/apply_offset_90_yaw', True)
    self.auto_calculate_ref = rospy.get_param(ns+'/auto_calculate_ref', True)
    
    # Subscriber and publisher
    rospy.Subscriber(topic_gps, NavSatFix, self.gps_callback)
    rospy.Subscriber(topic_reset, Empty, self.reset_calibration)
    self.enu_pub = rospy.Publisher(topic_publish, PoseWithCovarianceStamped, queue_size=5)

    if not self.auto_calculate_ref:
      # Create ecef references
      self.ecef_ref = self.geodetic_to_ecef(latitude_ref, longitude_ref, altitude_ref)
      # Create array reference geodetic
      self.LatLonAlt = [np.deg2rad(latitude_ref), np.deg2rad(longitude_ref), altitude_ref]
    
    # Variable for calibrate
    self.lat_array = []
    self.lon_array = []
    self.alt_array = []
    self.finish_calibrate = False
  
  def reset_calibration(self, empty):
    ''' reset calibration data '''
    self.lat_array = []
    self.lon_array = [] 
    self.alt_array = []
    self.finish_calibrate = False
    rospy.loginfo('Gps To Enu has been reseted ')

  def gps_callback(self, gps_data):
    ''' Callback for Gps with NavSatFix '''
    lat = gps_data.latitude
    lon = gps_data.longitude
    alt = gps_data.altitude
    
    if self.finish_calibrate:
      #self.LatLonAlt = [np.deg2rad(lat), np.deg2rad(lon), alt]
      # Convert to ecef coordinates
      ecef = self.geodetic_to_ecef(lat, lon, alt)
      # Convert to enu coordinates
      enu = self.ecef_to_enu(ecef[0], ecef[1], ecef[2])
      if self.apply_offset_yaw:
        enu[0],enu[1] = self.apply_offset_90_deg_clockwise(enu[0], enu[1])
      # Publish enu
      self.publish_enu_position(enu[0], enu[1], enu[2])
    else: # Calculate ref
      self.lat_array.append(lat)
      self.lon_array.append(lon)
      self.alt_array.append(alt)
      if len(self.lat_array) > 10:
        latitude_ref = np.mean(self.lat_array)
        longitude_ref = np.mean(self.lon_array)
        altitude_ref = np.mean(self.alt_array)
        self.ecef_ref = self.geodetic_to_ecef(latitude_ref, longitude_ref, altitude_ref)
        self.LatLonAlt = [np.deg2rad(latitude_ref), np.deg2rad(longitude_ref), altitude_ref]
        self.finish_calibrate = True
        rospy.loginfo('Finish calibrate lat, lon and alt references')

  def geodetic_to_ecef(self, lat, lon, alt):
    ''' Convert from geodetic coordinates (latitude, longitude, altitude) to ecef coordinates (XYZ from center of earth) '''
    # Constants (WGS-84)
    RAD_ECU = 6378137.0
    ECCEN = 0.081819190842622
    
    # Convert degrees to radians
    lat_rad = np.deg2rad(lat)
    lon_rad = np.deg2rad(lon)
    
    # Radius of curvature
    N = RAD_ECU / np.sqrt(1 - ECCEN**2 * np.sin(lat_rad)**2)
    
    # ECEF coordinates
    X = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
    Y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
    Z = (N * (1 - ECCEN**2) + alt) * np.sin(lat_rad)
    
    return ([X, Y, Z])
  
  def ecef_to_enu(self, x_ecef, y_ecef, z_ecef):
    ''' Convert from ecef coordinates (XYZ from center of earth) to enu coordinates (XYZ local based reference) '''
    # Difference betweeen new coordinates with reference
    dx_ecef = x_ecef - self.ecef_ref[0]
    dy_ecef = y_ecef - self.ecef_ref[1]
    dz_ecef = z_ecef - self.ecef_ref[2]
    # Transform matrix from ECEF to ENU
    # Source: https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates
    transform = np.array([
      [-np.sin(self.LatLonAlt[1]), np.cos(self.LatLonAlt[1]), 0],
      [-np.sin(self.LatLonAlt[0]) * np.cos(self.LatLonAlt[1]), -np.sin(self.LatLonAlt[0]) * np.sin(self.LatLonAlt[1]), np.cos(self.LatLonAlt[0])],
      [np.cos(self.LatLonAlt[0]) * np.cos(self.LatLonAlt[1]), np.cos(self.LatLonAlt[0]) * np.sin(self.LatLonAlt[1]), np.sin(self.LatLonAlt[0])]
    ])
    
    # Transform to ENU coordinates
    ENU = transform @ np.array([dx_ecef, dy_ecef, dz_ecef])
    
    return ENU
  
  def apply_offset_90_deg_clockwise(self, enu_x, enu_y):
    ''' Apply offset in Z axis '''
    # Rotation matrix
    rotation_matrix = np.array([
      [0, 1],
      [-1, 0]
    ])
    en_rotated = rotation_matrix @ np.array([enu_x, enu_y])
    return en_rotated
  
  def publish_enu_position(self, x, y, z):
    ''' Create msg and publish '''
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = self.frame_id
    
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    
    self.enu_pub.publish(msg)
    