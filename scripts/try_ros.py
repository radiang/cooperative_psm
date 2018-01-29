#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import numpy as np
import geometry_msgs.msg as gm
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from gazebo_msgs.srv import ApplyBodyWrench


if __name__ == '__main__':
    rospy.init_node("Force_node",anonymous=True)
    rospy.wait_for_service('gazebo/apply_body_wrench')
    get = rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)

    wrench = gm.Wrench() 
    wrench.force.x = 100
    wrench.force.y = 0
    wrench.force.z = 0
    wrench.torque.x = 0
    wrench.torque.y = 0
    wrench.torque.z = 0 
    try: 
    	get(body_name='PSM1::tool_wrist_link', wrench=wrench,duration = rospy.Duration(.5))

    except rospy.ServiceException as e:
    	print e
    #rospy.Subscriber('/psm/poses', gm.PoseStamped, self.handle_worldpose)
    # a= rospy.Publisher('/psm/poses', gm.PoseStamped,queue_size=1)
    # r = rospy.Rate(800)
    # message = gm.PoseStamped()
    # trans = [0]*3
