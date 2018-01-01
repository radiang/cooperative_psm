#!/usr/bin/env python 

import roslib
import numpy as np
import math
import time
import sys 
import rospy
import tf
from tf import TransformListener
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


from psm_coop import coop

if __name__ == '__main__':
	name = rospy.get_param('/name')
	num = rospy.get_param('/number')
	print(name)
	print(num)
	a = coop.mainframe(name, num)

	rospy.sleep(2)

	print('READY TO START')
	while not rospy.is_shutdown():
		#try: 
		a.run()
		for i in range(num):
			if (i>0):
			 	a.make_object_v(i)

			#print('amrunning')

		#except:
		#	continue        
	rospy.spin()