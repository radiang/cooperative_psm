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
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import ApplyBodyWrench

class environment(object):
    def __init__(self,name,num,k):
        self.num=num
        self.name=name
        self.init_pose = np.ones((self.num,3))
        self.new_pose = np.ones((self.num,3))
        self.init_object = np.ones((self.num,3))
        self.new_object = np.ones((self.num,3))
        self.change = np.ones((self.num,3))
        self.force = np.ones((4self.num,3,3))
        self.k=k
        self.namedict = {'one':'PSM1', 'two':'PSM2', 'three':'PSM3', 'four':'PSM4'}
        self.rot_tool = [None]*4

        self.initial_object()

    def initial_object(self):
        for i in range(self.num):
            (self.init_pose[i], self.rot_tool) = self.gazebo_service_call(self.namedict[self.name[i]]+'::tool_wrist_link','')
            
            self.init_object[i]=self.cyclic_substraction(self.init_pose,self.init_object,i,self.num-1)

    def gazebo_service_call(self,x,y):
        rospy.wait_for_service('/gazebo/get_link_state')
        try:
            get = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            resp1 = get(x,y)
            pos = [0]*3
            rot = [0]*4
            pos[0] = resp1.link_state.pose.position.x
            pos[1] = resp1.link_state.pose.position.y
            pos[2] = resp1.link_state.pose.position.z

            rot[0] = resp1.link_state.pose.orientation.x
            rot[1] = resp1.link_state.pose.orientation.y
            rot[2] = resp1.link_state.pose.orientation.z
            rot[3] = resp1.link_state.pose.orientation.w

            return pos , rot
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def gazebo_force_call(self,force):
        get = rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)
        wrench = gm.Wrench() 
        wrench.force.x = force[0]
        wrench.force.y = force[1]
        wrench.force.z = force[2]
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0 
        body_name = self.namedict[self.name[i]]+'::tool_wrist_link'

        try: 
            get(body_name=body_name, wrench=wrench,duration = rospy.Duration(.5))

        except rospy.ServiceException as e:
            print e
            
    def cyclic_substraction(self,x,y,num,num_max):
        if (num!=num_max):
            y[num]=x[num]-x[num-1]
        else: 
            y[num]=x[num]-x[num_max]

        return y[num]

    def cyclic_numbering(self,x,num_max):
        if (x>num_max):
            x = 0
        else:
        return x 

    def calculate_new_object(self): 
        for i in range(self.num):
            (self.new_pose[i], self.rot_tool) = self.gazebo_service_call(self.namedict[self.name[i]]+'::tool_wrist_link','')
            self.new_object[i]=self.cyclic_substraction(self.new_pose,self.new_object,i,self.num-1)

    def calculate_force(self): 
        for i in range(self.num):
            #Should use neighbor algorithm, right now neighbors are sequential i's
            j=cyclic_numbering(i+1,self.num)
            self.force[i][0]=self.k*(self.new_object[i]-self.init_object[i])*np.divide(self.new_object[i],np.linag.norm(self.new_object[i]))
            self.force[i][1]=-self.k*(self.new_object[j]-self.init_object[j])*np.divide(self.new_object[j],np.linag.norm(self.new_object[j]))
            self.force[i][2]=self.force[i][0]+self.force[i][1]

if __name__ == '__main__':
    rospy.init_node("object_node",anonymous=True)
    rospy.wait_for_service('gazebo/apply_body_wrench')

    name = rospy.get_param('/name')
    num = rospy.get_param('/number')
    gazebo_on = rospy.get_param('/gazebo_on')
    namedict = {'one':'PSM1', 'two':'PSM2', 'three':'PSM3', 'four':'PSM4'}
    
    k=10

    env = environment(name,num,k)    
    r = rospy.Rate(1600)

    while not rospy.is_shutdown():          
        env.calculate_new_object()
        env.calculate_force()
        for i in range(num):
            env.gazebo_force_call(env.force[i][3])
        r.sleep()
        
