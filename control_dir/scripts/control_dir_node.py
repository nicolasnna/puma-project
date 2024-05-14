#!/usr/bin/env python3

from control_dir.control_dir import *

control_dir = ControlDir()

if __name__ == '__main__':
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if not control_dir.isCalibrated():
            control_dir.calibrateDirection()
        else:
            control_dir.controlDirection()
        rate.sleep()