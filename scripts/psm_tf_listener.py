#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg as gm
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from gazebo_msgs.srv import GetLinkState

if __name__ == '__main__':
    rospy.init_node('tf_psm')

    listener = tf.TransformListener()

    #name = ['one', 'two']
    #number = len(name)

    name = rospy.get_param('/name')
    num = rospy.get_param('/number')
    gazebo_on = rospy.get_param('/gazebo_on')
    namedict = {'one':'PSM1', 'two':'PSM2', 'three':'PSM3', 'four':'PSM4'}
    #a = [None]*number
    # for i in range(number):

    #     a[i]= rospy.Publisher('/psm/poses/'+name[i], gm.PoseStamped,queue_size=1)
    #     rate=rospy.Rate(10)
    a= rospy.Publisher('/psm/poses', gm.PoseStamped,queue_size=1)
    r = rospy.Rate(800)
    message = gm.PoseStamped()
    trans = [0]*3

    while not rospy.is_shutdown():
        for i in range(num):

            try:
                if (gazebo_on == 0):   
                    (trans,rot) = listener.lookupTransform('/world' ,'/'+name[i]+'_tool_tf', rospy.Time(0))
                #print(trans)
                else:
                    get = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
                    resp1 = get(namedict[name[i]]+'::tool_wrist_sca_shaft_link','')

                    trans[0]=resp1.link_state.pose.position.x
                    trans[1]=resp1.link_state.pose.position.y
                    trans[2]=resp1.link_state.pose.position.z

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            message.header.frame_id = name[i]
            message.pose.position.x = trans[0]
            message.pose.position.y = trans[1]
            message.pose.position.z = trans[2]
            a.publish(message)
            r.sleep()
        
