#!/usr/bin/env python
 
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import numpy as np
import geometry_msgs.msg as gm
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class gazebo_wrapper():
    def __init__(self):
        self.name = rospy.get_param('/name')
        self.num = rospy.get_param('/number')
        self.gazebo_on = rospy.get_param('/gazebo_on')
        self.namedict = {'one':'PSM1', 'two':'PSM2', 'three':'PSM3', 'four':'PSM4'}
        self.numdict = {'one':0, 'two':1, 'three':2, 'four':3}

        rospy.init_node('gazebo_controllerwrapper', anonymous=True)


        self.j=[]
        self.j.append([None]*3)
        self.j.append([None]*3)
        self.j.append([None]*3)

        for i in range(self.num):
            rospy.Subscriber('/' + self.name[i], JointState, self.callback)
            self.j[i][0]=rospy.Publisher('/dvrk_psm/'+self.namedict[self.name[i]]+'/outer_yaw_joint/SetPositionTarget',Float64, queue_size=10)
            self.j[i][1]=rospy.Publisher('/dvrk_psm/'+self.namedict[self.name[i]]+'/outer_pitch_joint_1/SetPositionTarget',Float64, queue_size=10)
            self.j[i][2]=rospy.Publisher('/dvrk_psm/'+self.namedict[self.name[i]]+'/outer_insertion_joint/SetPositionTarget',Float64, queue_size=10)
        rospy.spin()

    def callback(self,data):
        self.j[self.numdict[data.header.frame_id]][int(data.velocity[0])].publish(data.position[0])
        
if __name__ == '__main__':
    a = gazebo_wrapper()