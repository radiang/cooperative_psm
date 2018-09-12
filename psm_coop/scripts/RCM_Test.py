#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg as gm
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from gazebo_msgs.srv import GetLinkState
import numpy as np 
import csv


def talker(): 
    rospy.init_node('other')
    listener = tf.TransformListener()

    a = rospy.Publisher('/one', JointState, queue_size=10)
    b = rospy.Publisher('two',JointState,queue_size=10) 
    r = rospy.Rate(500)
    message = JointState()
    message2 = JointState()

    message.header = Header()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = 'olalala'
    message.name = ['one_outer_yaw_joint']
    message.position = [0.0]
    message.velocity = []
    message.effort = []

    message2.header = Header()
    message2.header.stamp = rospy.Time.now()
    message2.header.frame_id = 'ol'
    message2.name = ['one_outer_pitch_joint_1']
    message2.position = [0.0]
    message2.velocity = []
    message2.effort = []



    trans = [0]*3
    dummy_r =[0]*3

    count = 0
    flag = 0
    string = [None]*3

    z = 50
    q1 = np.linspace(-1.50,1.50,num=z)
    q2 = np.linspace(-.75,.75,num=z)

    j = 0
    i = 0
    df = 0

    qdata1 = np.array([[0,0,0]])
    qdata2 = np.array([[0,0,0,0]])

    for j in range(z):
        for i in range(z):    
            message.position = [q1[j]]
            message2.position = [q2[i]]
            
            print('THIS SHIT:', j, i )

            for df in range(8):
                a.publish(message)
                rospy.sleep(0.001)
                b.publish(message2)

            try:
                (trans,dummy_r) = listener.lookupTransform('/world' ,'/one_tool_main_link', rospy.Time(0))
                (dummy,rot) = listener.lookupTransform('/world' ,'/one_tool_main_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            qdata1 = np.append(qdata1,[trans],axis=0)
            qdata2 = np.append(qdata2,[rot],axis=0)

            print(message.position, message2.position)
            print(trans)
            print(rot)
            r.sleep()  

    for d in range(4):
        (pitch, rot_pitch) = listener.lookupTransform('/world' ,'/one_outer_pitch_front_link', rospy.Time(0))

    with open('translations_link1.csv', 'wb') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(0,len(qdata1)):
            wr.writerow(qdata1[i])
    with open('rotations_world.csv', 'wb') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(0,len(qdata2)):
            wr.writerow(qdata2[i])

    with open('pitch_front_position.csv', 'wb') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(0,len(pitch)):
            wr.writerow(pitch)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
