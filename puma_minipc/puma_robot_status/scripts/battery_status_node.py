#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState

''' Valores de bateria 60V 17S Li-on '''
# battery_chart = [
#     (71.4, 100), (71.2, 99), (71.0, 98), (70.8, 97), (70.6, 96),
#     (70.4, 95), (70.2, 94), (70.0, 93), (69.8, 92), (69.6, 91),
#     (69.4, 90), (69.2, 89), (69.0, 88), (68.7, 87), (68.5, 86),
#     (68.3, 85), (68.1, 84), (67.9, 83), (67.7, 82), (67.5, 81),
#     (67.3, 80), (67.1, 79), (66.9, 78), (66.7, 77), (66.5, 76),
#     (66.3, 75), (66.1, 74), (65.9, 73), (65.7, 72), (65.5, 71),
#     (65.3, 70), (65.1, 69), (64.9, 68), (64.7, 67), (64.5, 66),
#     (64.3, 65), (64.1, 64), (63.9, 63), (63.6, 62), (63.4, 61),
#     (63.2, 60), (63.0, 59), (62.8, 58), (62.6, 57), (62.4, 56),
#     (62.2, 55), (62.0, 54), (61.2, 50), (61.0, 49), (60.8, 48),
#     (60.6, 47), (60.4, 46), (60.2, 45), (60.0, 44), (59.8, 43),
#     (59.6, 42), (59.4, 41), (59.2, 40), (59.0, 39), (58.8, 38),
#     (58.5, 37), (58.3, 36), (58.1, 35), (57.9, 34), (57.7, 33),
#     (57.5, 32), (57.3, 31), (57.1, 30), (56.9, 29), (56.7, 28),
#     (56.5, 27), (56.3, 26), (56.1, 25), (55.9, 24), (55.7, 23),
#     (55.5, 22), (55.3, 21), (55.1, 20), (54.9, 19), (54.7, 18),
#     (54.5, 17), (54.3, 16), (54.1, 15), (53.9, 14), (53.7, 13),
#     (53.4, 12), (53.2, 11), (53.0, 10), (52.8, 9), (52.6, 8),
#     (52.4, 7), (52.2, 6), (52.0, 5), (51.8, 4), (51.6, 3),
#     (51.4, 2), (51.2, 1)
# ]
battery_chart = [
    (71.4, 100.0), (71.2, 98.3), (71.0, 96.6), (70.8, 94.8), (70.6, 93.1),
    (70.4, 91.4), (70.2, 89.7), (70.0, 87.9), (69.8, 86.2), (69.6, 84.5),
    (69.4, 82.8), (69.2, 81.0), (69.0, 79.3), (68.7, 76.7), (68.5, 75.0),
    (68.3, 73.3), (68.1, 71.6), (67.9, 69.8), (67.7, 68.1), (67.5, 66.4),
    (67.3, 64.7), (67.1, 62.9), (66.9, 61.2), (66.7, 59.5), (66.5, 57.8),
    (66.3, 56.0), (66.1, 54.3), (65.9, 52.6), (65.7, 50.9), (65.5, 49.1),
    (65.3, 47.4), (65.1, 45.7), (64.9, 44.0), (64.7, 42.2), (64.5, 40.5),
    (64.3, 38.8), (64.1, 37.1), (63.9, 35.3), (63.6, 32.8), (63.4, 31.0),
    (63.2, 29.3), (63.0, 27.6), (62.8, 25.9), (62.6, 24.1), (62.4, 22.4),
    (62.2, 20.7), (62.0, 19.0), (61.2, 12.1), (61.0, 10.3), (60.8, 8.6),
    (60.6, 6.9), (60.4, 5.2), (60.2, 3.4), (60.0, 1.7), (59.8, 0.0)
]
voltajes = [par[0] for par in battery_chart]
porcentajes = [par[1] for par in battery_chart]

def battery_cb(msg: Float32):
  voltage_battery = msg.data
  battery_msgs = convert_voltage_to_battery(voltage_battery)
  publish_battery_status(battery_msgs)
  
def convert_voltage_to_battery(voltage: float):
  batery = BatteryState()
  batery.voltage = voltage
  if voltage >= voltajes[0]:
    batery.percentage = porcentajes[0]
  elif voltage <= voltajes[-1]:
    batery.percentage = porcentajes[-1]
  else:
    for i in range(1, len(voltajes)):
      if voltage < voltajes[i-1] and voltage >= voltajes[i]:
        batery.percentage = porcentajes[i]
        break
  return batery

def publish_battery_status(battery_status = BatteryState):
  pub = rospy.Publisher('/puma/sensors/battery/status', BatteryState, queue_size=10)
  pub.publish(battery_status)
    
def main():
  rospy.init_node('battery_status_node')
  rospy.Subscriber('/puma/sensors/battery/raw_72v', Float32, battery_cb)
  rospy.spin()
  
if __name__ == '__main__':
  main()