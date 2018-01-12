#!/usr/bin/env python
 
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg as gm
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
if __name__ == '__main__':
    rospy.init_node('gazebo_joint_controller')

    a= rospy.Publisher('/psm/poses', gm.PoseStamped,queue_size=1)


    r = rospy.Rate(800)
    message = gm.PoseStamped()
    while not rospy.is_shutdown():
        for i in range(num):

            try:
                #(trans,rot) = listener.lookupTransform('/world' ,'/' + name[i] + '_tool_wrist_sca_shaft_link', rospy.Time(0))
                (trans,rot) = listener.lookupTransform('/world' ,'/'+name[i]+'_tool_tf', rospy.Time(0))
                #print(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            message.header.frame_id = name[i]
            message.pose.position.x = trans[0]
            message.pose.position.y = trans[1]
            message.pose.position.z = trans[2]

            #print(trans,rospy.Time(0))
            # cmd = geometry_msgs.msg.Twist()
            # cmd.linear.x = trans[0]
            # cmd.linear.y = trans[1]
            # cmd.linear.z = trans[2]
            a.publish(message)
            #print(message)
            r.sleep()
