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
    b = [None]*num

    for i in range(num):
        b[i]= rospy.Publisher('/'+ name[i]+ '/poses', gm.Point,queue_size=1)
        
    r = rospy.Rate(800)
    message = gm.PoseStamped()
    n_msg = gm.Point()

    trans = [0]*3
    count = 0
    flag = 0
    string = [None]*3
    while not rospy.is_shutdown():
        for i in range(num):

            try:
                if (gazebo_on == 0):   
                    #(trans,rot) = listener.lookupTransform('/world' ,'/'+name[i]+'_tool_tf', rospy.Time(0))
                    (trans,rot) = listener.lookupTransform('/'+name[0]+'_remote_center_link' ,'/'+name[i]+'_tool_tf', rospy.Time(0))
                #print(trans)
                else:
                    get = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
                    #resp1 = get(namedict[name[i]]+'::tool_wrist_sca_shaft_link','')
                    resp1 = get(namedict[name[i]]+'::tool_wrist_link',namedict[name[0]]+'::remote_center_link')

                    trans[0]=resp1.link_state.pose.position.x
                    trans[1]=resp1.link_state.pose.position.y
                    trans[2]=resp1.link_state.pose.position.z

                if (flag ==0):
                    string[0] = name[i] + '_init_tool/x'
                    string[1] = name[i] + '_init_tool/y'
                    string[2] = name[i] + '_init_tool/z'       
                    
                    rospy.set_param(string[0], trans[0])
                    rospy.set_param(string[1], trans[1])
                    rospy.set_param(string[2], trans[2])

                if (count>150*10):
                    flag =1

                count=count+1

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            message.header.frame_id = name[i]
            message.header.stamp = rospy.Time.now() 
            message.pose.position.x = trans[0]
            message.pose.position.y = trans[1]
            message.pose.position.z = trans[2]
            message.pose.orientation.x = count
            message.pose.orientation.y = flag
            a.publish(message)

            n_msg.x = trans[0] 
            n_msg.y = trans[1]
            n_msg.z = trans[2]
            b[i].publish(n_msg)
            r.sleep()
        
