#!/usr/bin/env python  
import roslib
import rospy
import copy
import time
import math
import tf
import numpy as np
import geometry_msgs.msg as gm
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import ApplyBodyWrench
from gazebo_msgs.msg import LinkStates

class Timer(object):
    def __init__(self, name=None):
        self.name = name

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        t = (time.time() - self.tstart)
        x = float(1)/t

        if self.name:
            x = 0
            #print '[%s]' % self.name,
        #print 'Time Elapsed: %s Hz: %s' % (t,x)

class environment(object):
    def __init__(self,name,num,k,c):
        self.num=num
        self.name=name
        self.init_pose = np.ones((self.num,3))
        self.new_pose = np.ones((self.num,3))
        self.init_object = np.ones((self.num,3))
        self.new_object = np.ones((self.num,3))
        self.new_object_min1 = np.ones((self.num,3))
        self.change = np.ones((self.num,3))
        self.force = np.ones((self.num,3,3))
        self.k = k
        self.c = c
        self.namedict = {'one':'PSM1', 'two':'PSM2', 'three':'PSM3', 'four':'PSM4'}
        self.rot_tool = [None]*4

        rospy.Subscriber('/gazebo/link_states', LinkStates, self.callback)

        self.initial_object()

    def callback(self,msg):
        for i in range(self.num):
            j = msg.name.index('dvrk_psm::'+self.namedict[self.name[i]]+'::tool_wrist_link')

            self.new_pose[i][0] = msg.pose[j].position.x
            self.new_pose[i][1] = msg.pose[j].position.y
            self.new_pose[i][2] = msg.pose[j].position.z

    def initial_object(self):
        for i in range(self.num):
            (self.init_pose[i], self.rot_tool) = self.gazebo_service_call(self.namedict[self.name[i]]+'::tool_wrist_link','world')
            #(self.init_pose[i], self.rot_tool) = self.gazebo_service_call(self.namedict[self.name[i]]+'::tool_wrist_link',self.namedict[self.name[0]]+'::base_link')
        if(self.num==2):
            for i in range(self.num):    
                self.init_object[i]=self.cyclic_substraction(self.init_pose,self.init_object,i,self.num-1)
                print('init',self.init_pose[i])
        else: 
            (self.init_object[0],self.init_object[1],self.init_object[2]) = self.make_centroid(self.init_pose[0],self.init_pose[1],self.init_pose[2])

        print('init',self.init_pose[i])
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

    def gazebo_force_call(self,force,i):

        get = rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)
        wrench = gm.Wrench() 
        wrench.force.x = force[0]
        wrench.force.y = force[1]
        wrench.force.z = force[2]
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0 
        body_name = self.namedict[self.name[i]]+'::tool_wrist_link'
        ref_name = self.namedict[self.name[0]]+'::base_link'
        #print(wrench)
        try: 
            get(body_name=body_name,reference_frame='world',wrench=wrench,duration = rospy.Duration(.4))

        except rospy.ServiceException as e:
            print e
            
    def cyclic_substraction(self,x,y,num,num_max):
        if (num==0):
            y[num]=x[num_max]-x[num]
        else: 
            y[num]=x[num-1]-x[num]
        return y[num]

    def cyclic_numbering(self,x,num_max):
        if (x>num_max):
            x = 0
        else:
            x = x
        return x 

    def make_centroid(self,pose1,pose2,pose3):
        #print(self.worldpose[1],self.worldpose[0])
        if (self.num == 2):
            self.object_v[0] = (np.array(self.worldpose[1])-np.array(self.worldpose[0]))/2
            self.object_v[1] = (np.array(self.worldpose[0])-np.array(self.worldpose[1]))/2
        
        else:    
            v1 = np.array(pose2)-np.array(pose1)
            v2 = np.array(pose3)-np.array(pose1)
            b = v2 - v1

            c_m = (v1+b/2)*2/3

            zero = c_m
            one = c_m - v1
            two = c_m - v2

            return zero, one, two


    def calculate_new_object(self): 
        for i in range(self.num):
            with Timer('service_call'):
            #(self.new_pose[i], self.rot_tool) = self.gazebo_service_call(self.namedict[self.name[i]]+'::tool_wrist_link','world')
            #(self.new_pose[i], self.rot_tool) = self.gazebo_service_call(self.namedict[self.name[i]]+'::tool_wrist_link',self.namedict[self.name[0]]+'::base_link')
                self.new_object_min1[i] = copy.copy(self.new_object[i])

            if (self.num==2):        
                self.new_object[i]=self.cyclic_substraction(self.new_pose,self.new_object,i,self.num-1)
        
        if(self.num>2):
            (self.new_object[0],self.new_object[1],self.new_object[2]) = self.make_centroid(self.new_pose[0],self.new_pose[1],self.new_pose[2])
    
    def calculate_force(self): 
        for i in range(self.num):
            #Should use neighbor algorithm, right now neighbors are sequential i's
            # if(self.num>2):
            #     j=self.cyclic_numbering(i+1,self.num-1)

            #     self.force[i][0]=self.k*(self.new_object[i]-self.init_object[i])*np.divide(self.new_object[i],np.linalg.norm(self.new_object[i]))
            #     self.force[i][1]=-self.k*(self.new_object[j]-self.init_object[j])*np.divide(self.new_object[j],np.linalg.norm(self.new_object[j]))
            #     self.force[i][2]=self.force[i][0]+self.force[i][1]
            # else: 
            k_diff = self.k*(np.linalg.norm(self.new_object[i])-np.linalg.norm(self.init_object[i]))
            c_diff = self.c*(np.linalg.norm(self.new_object[i])-np.linalg.norm(self.new_object_min1[i]))
            #print('c_diff',c_diff)
            self.force[i][0]=(k_diff+c_diff)*np.divide(self.new_object[i],np.linalg.norm(self.new_object[i]))
 
            #print(i, self.force[i][0])
    def message_making(self,force,i):
        msg = gm.WrenchStamped()

        msg.wrench.force.x = force[0]
        msg.wrench.force.y = force[1]
        msg.wrench.force.z = force[2]
        msg.wrench.torque.x = self.calculate_force_dir(i)*np.linalg.norm(np.array(force))
        msg.wrench.torque.y = 0
        msg.wrench.torque.z = 0 #+1 : pulling force, -1: pushing  force

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id= str(i)
        return msg

    def calculate_force_dir(self,i):
        x = np.linalg.norm(self.new_object[i])-np.linalg.norm(self.init_object[i])
        if (x>0):
            y = 1
        else:
            y = -1
        return y 

if __name__ == '__main__':
    rospy.init_node("object_node",anonymous=True)
    rospy.wait_for_service('gazebo/apply_body_wrench')

    name = rospy.get_param('/name')
    num = rospy.get_param('/number')
    gazebo_on = rospy.get_param('/gazebo_on')
    namedict = {'one':'PSM1', 'two':'PSM2', 'three':'PSM3', 'four':'PSM4'}

    p = [None]*num
    for i in range(num):
        p[i]=rospy.Publisher('/psm_sense/'+namedict[name[i]]+'/tool_forces',gm.WrenchStamped, queue_size=10)

    
    k=10000 #stiffness of suture material from ref[] N/m
    c = 10 #damping of object N/m2

    env = environment(name,num,k,c)    
    r = rospy.Rate(4000)
    rospy.sleep(1)
    j = 0
    while not rospy.is_shutdown():          
        
        with Timer('calc_new_object'):
            env.calculate_new_object()
        with Timer('calc_force'):
            env.calculate_force()
        if (j == 0):
            print('new_pose',env.new_pose)
            print('force',env.force)
            #rospy.sleep(1)
        #print('init_object',env.init_object)
        #print('new_object',env.new_object)
        #print('force',env.force)

        for i in range(num):
            if(num>2):
                with Timer('force_call'):
                    env.gazebo_force_call(env.force[i][0],i)
                msg = env.message_making(env.force[i][0],i)
                with Timer('publish'):
                    p[i].publish(msg)
            else:
                env.gazebo_force_call(env.force[i][0],i)
                msg = env.message_making(env.force[i][0],i)
                p[i].publish(msg)

        j = j + 1
        r.sleep()
        
