#!/usr/bin/env python3
import roslaunch
import os

current_path = os.getcwd()
path_map = current_path + "/map-c00.yaml"

package = 'map_server'
name = 'map_server'
type = 'map_server'
args = path_map

node = roslaunch.core.Node(package,type,name,args=args)


launch = roslaunch.scriptapi.ROSLaunch()
launch.start()
process = launch.launch(node)
import time
while True:
  time.sleep(2)
