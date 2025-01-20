import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PlanManager:
  def __init__(self, ns):
    self._plan = []
    self._plan_remaining = []
    self._in_navigation = False
    self.init_publishers(ns)
    self.start_subscriber(ns)
    
  def init_publishers(self, ns):
    self.plan_list_pub = rospy.Publisher(ns+'/plan_info', Path, queue_size=10)
    self.plan_remaining_pub = rospy.Publisher(ns+'/remaining', Path, queue_size=5)
    
  def start_subscriber(self, ns):
    self.add_plan_sub = rospy.Subscriber(ns+'/add', Path, self.add_plan_callback)
    self.plan_update_sub = rospy.Subscriber(ns+'/update', Path, self.update_plan_callback)

  def update_plan_callback(self, plan):
    self._plan_remaining = plan.poses
    
  def add_plan_callback(self, plan):
    self._plan = plan.poses
    
  def publish_path(self, publisher, plan):
    plan_msg = Path()
    plan_msg.poses = plan
    plan_msg.header.stamp = rospy.Time.now()
    publisher.publish(plan_msg)
    
  def publish_status(self):
    self.publish_path(self.plan_list_pub, self._plan)
    self.publish_path(self.plan_remaining_pub, self._plan_remaining)