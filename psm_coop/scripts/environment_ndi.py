#!/usr/bin/env python  
import roslib
import rospy
import copy
import time
import math

import numpy as np
import geometry_msgs.msg as gm
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header

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
    def __init__(self,name,num,k,c,enable):
        self.num=num
        self.name=name
        self.enable = enable
        
    
        self.k = k
        self.c = c
        self.namedict = {'one':'PSM1', 'two':'PSM2', 'three':'PSM3', 'four':'PSM4'}
        self.rot_tool = [None]*4


        self.count = 0

        rospy.Subscriber('/ndi/fiducials', PointCloud, self.callback)
        rospy.sleep(1)
        
        self.init_pose = np.ones((self.track_num,3))
        self.new_pose = np.ones((self.track_num,3))
        self.init_object = np.ones((self.track_num,3))
        self.new_object = np.ones((self.track_num,3))
        self.new_object_min1 = np.ones((self.track_num,3))
        self.change = np.ones((self.track_num,3))
        self.force = np.ones((self.track_num,3,3))

        self.initial_object()

    def callback(self,msg):
        
        if (self.count < 10):
            self.track_num = len(msg.points)
            self.tracker_pose = np.ones((self.track_num,3)) 

        for i in range(self.track_num):
            self.tracker_pose[i][0] = msg.points[i].x 
            self.tracker_pose[i][1] = msg.points[i].y 
            self.tracker_pose[i][2] = msg.points[i].z 

        self.count = self.count + 1
      
      
    def initial_object(self):
        self.init_pose = copy.copy(self.tracker_pose)
        print('init',self.init_pose)
        for i in range(self.track_num):
            if(self.track_num==2):           
                self.init_object[i] = self.init_pose[self.cyclic_numbering(i+1,self.track_num)] - self. init_pose[i]
                print('init_object',self.init_object[i])
        
        if(self.track_num>2): 
            (self.init_object[0],self.init_object[1],self.init_object[2]) = self.make_centroid(self.init_pose[0],self.init_pose[1],self.init_pose[2])

            
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

    def cyclic_numbering(self,x,num_max):
        if (x>num_max-1):
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
        self.new_pose = copy.copy(self.tracker_pose)
        self.new_object_min1 = copy.copy(self.new_object)
        for i in range(self.track_num):
            if (self.track_num==2):        
                self.new_object[i] = self.new_pose[self.cyclic_numbering(i+1,self.track_num)] - self.new_pose[i]
        if(self.track_num>2):
            (self.new_object[0],self.new_object[1],self.new_object[2]) = self.make_centroid(self.new_pose[0],self.new_pose[1],self.new_pose[2])
        
        #print('new_object',self.new_object)

    def calculate_force(self): 
        for i in range(self.num):
        
            k_diff = self.k*(np.linalg.norm(self.new_object[i])-np.linalg.norm(self.init_object[i]))
            c_diff = self.c*(np.linalg.norm(self.new_object[i])-np.linalg.norm(self.new_object_min1[i]))
            #print('c_diff',c_diff)
            self.force[i][0]=(k_diff+c_diff)*np.divide(self.new_object[i],np.linalg.norm(self.new_object[i]))
 
            print('force',i, self.force[i][0])
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

    #ame = rospy.get_param('/name')
    #num = rospy.get_param('/number')
    #gazebo_on = rospy.get_param('/gazebo_on')


    name = ['one','two']
    num = 2
    gazebo_on = 0
    enable = 0 
    namedict = {'one':'PSM1', 'two':'PSM2', 'three':'PSM3', 'four':'PSM4'}

    p = [None]*num
    for i in range(num):
        p[i]=rospy.Publisher('/psm_sense/'+namedict[name[i]]+'/tool_forces',gm.WrenchStamped, queue_size=10)

    k=1000 #stiffness of object N/m
    c = 10 #damping of object N/m2

    env = environment(name,num,k,c, enable)    
    r = rospy.Rate(4000)
    j = 0

    while not rospy.is_shutdown():          
        env.calculate_new_object()
        env.calculate_force()

        for i in range(env.track_num):
            msg = env.message_making(env.force[i][0],i)
            with Timer('publish'):
                p[i].publish(msg)
        r.sleep()
        # with Timer('calc_new_object'):
        #     env.calculate_new_object()
        # with Timer('calc_force'):
        #     env.calculate_force()
        # if (j == 0):
        #     print('new_pose',env.new_pose)
        #     print('force',env.force)
        #     #rospy.sleep(1)
        # #print('init_object',env.init_object)
        # print('new_object',env.new_object)
        # print('force',env.force)

        # for i in range(num):
        #     if(num>2):
        #         with Timer('force_call'):
        #             env.gazebo_force_call(env.force[i][0],i)
        #         msg = env.message_making(env.force[i][0],i)
        #         with Timer('publish'):
        #             p[i].publish(msg)
        #     else:
        #         env.gazebo_force_call(env.force[i][0],i)
        #         msg = env.message_making(env.force[i][0],i)
        #         p[i].publish(msg)

        # j = j + 1
        # r.sleep()
        
