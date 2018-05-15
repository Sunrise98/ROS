#!/usr/bin/env python
# license removed for brevity
import rospy
from msg_test.msg import test

"""
@param msg getting messege
print massege contens  
"""
def callback(msg):
	print "published!(ID: %d, name: %s)"%(msg.ID, msg.name)


def sub():
	"""
	call callback method when get the messeage named 'test_data'  
	"""
	rospy.init_node('sub', anonymous=True)

	rospy.Subscriber('test_data', test, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		sub()
	except rospy.ROSInterruptException: pass