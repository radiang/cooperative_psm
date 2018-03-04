#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg as gm
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from gazebo_msgs.srv import GetLinkState
def callback(msg):
    global pose
    pose = msg


if __name__ == '__main__':
    rospy.init_node('mtm_tf')

    listener = tf.TransformListener()

    hardware = 0
  
    a= rospy.Publisher('/psm/cmd_vel', gm.Twist,queue_size=1)
   
    r = rospy.Rate(10)
    message = gm.PoseStamped()
    n_msg = gm.Point()

    trans = [0]*3
    mtm_init = [0]*3
    mtm_diff = [0]*3
    count = 0

    if (hardware ==1):
        b=rospy.Subscriber('/mtml',gm.PoseStamped,callback)

    while not rospy.is_shutdown():
        try:
            if(hardware == 0):
                (trans,rot) = listener.lookupTransform('/right_base' ,'/right_wrist_yaw_link', rospy.Time(0))
                #print(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        if (hardware==0):
            if (count == 0):
                mtm_init[0] = trans[0]
                mtm_init[1] = trans[1]
                mtm_init[2] = trans[2]
            else:
                mtm_diff[2] = trans[2] - mtm_init[2]
                mtm_diff[1] = trans[1] - mtm_init[1]
                mtm_diff[0] = trans[0] - mtm_init[0]
        else :
            if (count == 0):
                mtm_init = trans
            else :
                mtm_diff[2] = trans[2] - mtm_init[2]
                mtm_diff[1] = trans[1] - mtm_init[1]
                mtm_diff[0] = trans[0] - mtm_init[0]

        count = count +1
        scale = -1
        twist = gm.Twist()
        twist.linear.x = scale*mtm_diff[0]; twist.linear.y = scale*mtm_diff[1]; twist.linear.z = -scale*mtm_diff[2]
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        a.publish(twist)

        print(twist)
        r.sleep()
        
