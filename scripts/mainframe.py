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
	gazebo_on = rospy.get_param('/gazebo_on')
	
	a = coop.mainframe(name, num,gazebo_on)
	r = rospy.Rate(1500)
	rospy.sleep(1)

	print('READY TO START')
	while not rospy.is_shutdown():
		for i in range(num):
			if (i>0):
			 	a.make_object_v(i)
		a.run()  
		r.sleep()
	#rospy.spin()