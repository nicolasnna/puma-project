import rospy
from geometry_msgs.msg import Twist
import curses

start_message=""""
Reading keyboard and publish cmd_vel (Twist)
-------------------------------------------
* Commands WASDX
* Press Q or Ctr+C to quit

W: Increment lineal velocity x in 0.2 m/s
S: Stop lineal and angular velocity
A: Decrement angular velocity z in 0.1 m/s
D: Increment angular velocity z in 0.1 m/s
X: Decrement lineal velocity z in 0.2 m/s
-------------------------------------------
"""

def start_curses():
    app = curses.initscr()  # Create a terminal window
    curses.noecho()         # Makes input invisible
    app.addstr(start_message)   # Print the start message
    return app

if __name__ == '__main__':
    try:
      rospy.init_node('teleop_keyboard')
      cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
      cmd_vel_msg = Twist()
      
      app = start_curses()
      
      while not rospy.is_shutdown():
        key = app.getkey()
        
        if key.lower() == 'w':
          cmd_vel_msg.linear.x += 0.2
        elif key.lower() == 'x':
          cmd_vel_msg.linear.x -= 0.2
        elif key.lower() == 'a':
          cmd_vel_msg.angular.z += 0.1
        elif key.lower() == 'd':
          cmd_vel_msg.angular.z -= 0.1
        elif key.lower() == 's':
          cmd_vel_msg.linear.x = 0
          cmd_vel_msg.angular.z = 0 
        elif key.lower() == 'q':
          curses.endwin()
          break      
          
        cmd_vel_pub.publish(cmd_vel_msg)
        rospy.Rate(10).sleep()
        
    except rospy.ROSInterruptException:
      curses.endwin()
      rospy.logwarn("Cerrando nodo 'teleop_keyboard' !!!")
        
    except Exception as e:
      curses.endwin()
      rospy.logerr("Error al ejecutar el nodo: %s", e)