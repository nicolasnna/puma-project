import rospy
import time

class ResendTopic:
  def __init__(self, topic_name, type_msgs, freq, new_topic_name):
    
    self._time_limit = 1 / freq
    self._type_msgs = type_msgs
    
    rospy.Subscriber(topic_name, type_msgs, self.resend_callback)
    self.msg_pub = rospy.Publisher(new_topic_name, type_msgs, queue_size=2)
    self.lastTime = time.time()
    self.msg_data = type_msgs()

  def resend_callback(self, msg):
    self.msg_data = msg
    
  def publish(self):
    time_now = time.time()
    if time_now - self.lastTime >= self._time_limit:
      self.msg_pub.publish(self.msg_data)
      self.lastTime = time_now