#!/usr/bin/env python
# license removed for brevity
import rospy
from msg_test.msg import test

def pub():
  rospy.init_node('pub', anonymous=True)
  pub = rospy.Publisher('test_data', test, queue_size=10)
  r = rospy.Rate(10) # 10hz

  msg = test()
  i = 0

  while not rospy.is_shutdown():
    msg.ID = i;
    msg.name = "Hello"
    pub.publish(msg)
    print "published!(ID: %d, name: %s)"%(msg.ID, msg.name)
    i += 1
    r.sleep() 

if __name__ == '__main__':
  try:
    pub()
  except rospy.ROSInterruptException: pass