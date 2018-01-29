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
        print('yes1')

        self.j=[]
        self.j.append([None]*4)
        self.j.append([None]*4)
        self.j.append([None]*4)


        for i in range(self.num):
            rospy.Subscriber('/' + self.name[i], JointState, self.callback)
            self.j[i][0]=rospy.Publisher('/dvrk_psm/'+self.namedict[self.name[i]]+'/outer_yaw_joint/SetPositionTarget',Float64, queue_size=10)
            self.j[i][1]=rospy.Publisher('/dvrk_psm/'+self.namedict[self.name[i]]+'/outer_pitch_joint_1/SetPositionTarget',Float64, queue_size=10)
            self.j[i][2]=rospy.Publisher('/dvrk_psm/'+self.namedict[self.name[i]]+'/outer_insertion_joint/SetPositionTarget',Float64, queue_size=10)
            self.j[i][3]=rospy.Publisher('/dvrk_psm/'+self.namedict[self.name[i]]+'/outer_roll_joint/SetPositionTarget',Float64, queue_size=10)
            
        print('yes2')
        

    def pupu(self):
        print("yes3")

        for i in range (15):
            for i in range(self.num):
                self.j[i][0].publish(0.0)
                self.j[i][1].publish(0.0)
                self.j[i][2].publish(0.15)
                self.j[i][3].publish(0.0)    
                print("yes4")
            rospy.sleep(0.1)
    def callback(self,data):
        if(int(data.velocity[0])!=2):
            self.j[self.numdict[data.header.frame_id]][int(data.velocity[0])].publish(data.position[0])
        #else:
         #   self.j[self.numdict[data.header.frame_id]][int(data.velocity[0])].publish(0.15)
        #self.j[0][3].publish(0.0)
        #self.j[1][3].publish(0.0)

if __name__ == '__main__':
    a = gazebo_wrapper()
    rospy.sleep(0.5)
    a.pupu()
    rospy.spin()
   
         