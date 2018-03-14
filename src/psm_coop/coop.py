#!/usr/bin/env python  
import roslib
import numpy as np
import math
import time
import sys 
import rospy
import tf
from tf import TransformListener
import geometry_msgs.msg as gm
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from gazebo_msgs.srv import GetLinkState

class psm(object):
    #global number
    number = 0
    def __init__(self, name, gazebo_on): 
        self.name = name
        self.namedict = {'one':'PSM1', 'two':'PSM2', 'three':'PSM3', 'four':'PSM4'}
        self.joint_names = [self.name + '_outer_yaw_joint', self.name + '_outer_pitch_joint_1', self.name + '_outer_insertion_joint']
        self.p_rcm = []
        self.p_tool = []
        self.p_init = []
        #self.worldpose = [] 
        self.rot_rcm = [None] * 4
        self.rot_tool = [None] * 4
        self.obj = [0.15, 0.15, 0.15]
        self.counter = 0

        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
                    
        while (not (self.p_rcm and self.p_tool)): 
            try:
                if (gazebo_on == 0):
                    (self.p_rcm, self.rot_rcm) = self.listener.lookupTransform('/world','/' + self.name + '_remote_center_link', rospy.Time(0))
                    (self.p_tool, self.rot_tool) = self.listener.lookupTransform('/' + self.name + '_remote_center_link','/' + self.name + '_tool_wrist_sca_shaft_link', rospy.Time(0))
                else: 
                    (self.p_rcm, self.rot_rcm) = self.gazebo_service_call(self.namedict[self.name]+'::remote_center_link','')
                    #(self.p_tool, self.rot_tool) = self.gazebo_service_call(self.namedict[self.name]+'::tool_wrist_sca_shaft_link',self.namedict[self.name]+'::remote_center_link')
                    (self.p_tool, self.rot_tool) = self.gazebo_service_call(self.namedict[self.name]+'::tool_wrist_link',self.namedict[self.name]+'::remote_center_link')
                   
                print('trying')              
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        self.matrix_rot_rcm = tf.transformations.quaternion_matrix(self.rot_rcm)
        self.inv_rot_rcm=tf.transformations.inverse_matrix(self.matrix_rot_rcm)

        self.p_init = np.copy(self.p_tool)
        print('/'+ self.name + '_rcm',self.p_rcm,self.rot_rcm)        
        print('/'+ self.name + '_tool',self.p_tool)
        #print(self.matrix_rot_rcm)
        rospy.sleep(1)

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

    def move(self,msg):
        self.p_tool[0] = self.p_tool[0]+msg.linear.x
        self.p_tool[1] = self.p_tool[1]+msg.linear.y
        self.p_tool[2] = self.p_tool[2]+msg.linear.z

    def move_abs(self,msg):
        self.p_tool[0] = self.p_init[0]+msg.linear.x
        self.p_tool[1] = self.p_init[1]+msg.linear.y
        self.p_tool[2] = self.p_init[2]+msg.linear.z

    # def handle_worldpose(self, msg):
    #     self.p_tool = [msg.linear.x, msg.linear.y,msg.linear.z]

    def get_pose(self):
        return self.p_tool



    def inverse_kinematic(self,x,y,z): 
        joint_angle=[0,0,0]
        joint_angle[0] = -math.atan(x/z)
        joint_angle[1] = -math.atan(y/math.sqrt(math.pow(x,2)+math.pow(z,2))) 
        joint_angle[2] = math.sqrt(math.pow(x,2)+math.pow(y,2)+math.pow(z,2))

        return joint_angle

    def message_making(self,string,data,j): 
        hello_str = JointState()
        hello_str.header = Header()
        hello_str.header.stamp = rospy.Time.now()
        hello_str.header.frame_id = self.name
        hello_str.name = [string]
        hello_str.position = [data]
        hello_str.velocity = [j]
        hello_str.effort = []

        return hello_str

    def send_transform(self): 
        
        self.br.sendTransform((self.p_rcm), (self.rot_rcm), rospy.Time.now(), self.name+"_rcm_tf", "world")
        self.br.sendTransform((self.p_tool), (0,0,0,1), rospy.Time.now(), self.name+"_tool_tf", self.name+"_rcm_tf")
        self.br.sendTransform((self.obj[0],self.obj[1],self.obj[2]), (0,0,0,1), rospy.Time.now(),self.name+'_test', self.name+"_tool_tf")

    def pu_publish(self): 
        data = self.inverse_kinematic(self.p_tool[0],self.p_tool[1],self.p_tool[2])
        
        #print(data)
        for i in range(len(data)):
            msg = self.message_making(self.joint_names[i],data[i])
            #print(msg)
            self.p.publish(msg)

 
            
class master(psm):
    def __init__(self,name, gazebo_on):
        psm.master = name
        psm.number +=1
        psm.__init__(self,name, gazebo_on)
        #print('done init',psm.master)


#Optional command, re-orient based off of centroid of object.
    def move(self, msg):
        #Linear Movement of Slave PSM
        new_vector = np.array([msg.linear.x, msg.linear.y, msg.linear.z,1])
        #print(new_vector)
        self.p_tool[0] = self.p_tool[0] + new_vector[0]
        self.p_tool[1] = self.p_tool[1] + new_vector[1]
        self.p_tool[2] = self.p_tool[2] + new_vector[2]

        #Angular Movement of PSM1 based off of centroid of object.
        rot=tf.transformations.euler_matrix(msg.angular.x,msg.angular.y,msg.angular.z, 'rxyz')
        rot_i=rot-np.identity(4)
        new_rotation=rot_i.dot(np.append(-self.obj,1))
            
        new_vector = np.array([new_rotation[0], new_rotation[1], new_rotation[2],1])
        self.p_tool[0] = self.p_tool[0] + new_vector[0]
        self.p_tool[1] = self.p_tool[1] + new_vector[1]
        self.p_tool[2] = self.p_tool[2] + new_vector[2]

        #Optional command, re-orient based on PSM1, Turn above lines Off 

    def move_force(self,msg):
        new_vector= self.obj[0:3]/np.linalg.norm(self.obj[0:3])*msg.linear.x
    
            
        self.p_tool[0]=self.p_tool[0]+new_vector[0]
        self.p_tool[1]=self.p_tool[1]+new_vector[1]
        self.p_tool[2]=self.p_tool[2]+new_vector[2]

    def set_object(self,ob):
    #    R= np.matrix.transpose(self.matrix_rot_rcm[0:3][0:3]) 
    #    x = np.matmul(R,ob)
    #    self.obj = np.array([x[0],x[1],x[2]])   
        self.obj = ob
        #print(np.linalg.norm(self.obj))

class slave(psm):
    def __init__(self,name,gazebo_on):
        
        psm.number +=1
        psm.__init__(self,name, gazebo_on)
  
        self.p_master = []
        self.rot_master = [None]*4

        print(' I got to slave')
        while (not self.p_master): 
            try:
                if (gazebo_on==0):
                    # ROT_MASTER IS R^MASTER_SLAVE: R MASTER TO SLAVE
                    (self.p_master,self.rot_master) = self.listener.lookupTransform('/' +psm.master+ '_remote_center_link','/'+self.name+'_remote_center_link', rospy.Time(0))
                else: 
                    #(self.p_master, self.rot_master) = self.gazebo_service_call(self.namedict[psm.master]+'::remote_center_link', self.namedict[self.name]+'::remote_center_link')
                    (self.p_master, self.rot_master) = self.gazebo_service_call(self.namedict[self.name]+'::remote_center_link', self.namedict[psm.master]+'::remote_center_link')
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        print('/'+ self.name + '_master',self.p_master)
        rospy.sleep(1)
        #print(self.name,self.rot_master, 'Here I go again')
        Re = tf.transformations.quaternion_matrix(self.rot_master)
        self.inv_rot_master=tf.transformations.inverse_matrix(Re)
        #print(self.inv_rot_master, 'Here I go again')


    def move(self, msg):
        #Linear Movement of Slave PSM
        new_vector = self.inv_rot_master.dot(np.array([msg.linear.x, msg.linear.y, msg.linear.z,1]))
        #print(new_vector)
        self.p_tool[0] = self.p_tool[0] + new_vector[0]
        self.p_tool[1] = self.p_tool[1] + new_vector[1]
        self.p_tool[2] = self.p_tool[2] + new_vector[2]

        #Angular Movement of PSM2 is based on centroid of object
        rot=tf.transformations.euler_matrix(msg.angular.x,msg.angular.y,msg.angular.z, 'rxyz')
        rot_i=rot-np.identity(4)
        new_vector=rot_i.dot(np.append(-self.obj,1))
            
        #new_vector = self.inv_rot_master.dot(np.array([new_rotation[0], new_rotation[1], new_rotation[2],1]))

        #Make Angular Movement of PSM2 based on master position of object 
        self.p_tool[0] = self.p_tool[0] + new_vector[0]
        self.p_tool[1] = self.p_tool[1] + new_vector[1]
        self.p_tool[2] = self.p_tool[2] + new_vector[2]

    def move_abs(self, msg):
        #Linear Movement of Slave PSM
        #print('Mothafuka')
        new_vector = self.inv_rot_master.dot(np.array([msg.linear.x, msg.linear.y, msg.linear.z,1]))
        #print(new_vector)
        self.p_tool[0] = self.p_init[0]+new_vector[0]
        self.p_tool[1] = self.p_init[1]+new_vector[1]
        self.p_tool[2] = self.p_init[2]+new_vector[2]

        #Angular Movement of PSM2 is based on centroid of object
        rot=tf.transformations.euler_matrix(msg.angular.x,msg.angular.y,msg.angular.z, 'rxyz')
        rot_i=rot-np.identity(4)
        new_rotation=rot_i.dot(np.append(self.obj,1))
            
        new_vector = self.inv_rot_master.dot(np.array([new_rotation[0], new_rotation[1], new_rotation[2],1]))

        # #Make Angular Movement of PSM2 based on master position of object 
        #self.p_tool[0] = self.p_init[0]+new_vector[0]
        #self.p_tool[1] = self.p_init[1]+new_vector[1]
        #self.p_tool[2] = self.p_init[2]+new_vector[2]



    def move_force(self,msg):
          #Force Movement of PSM2 based on centroid of object 
        new_vector= self.obj[0:3]/np.linalg.norm(self.obj[0:3])*msg.linear.x
        #new_vector = self.inv_rot_master.dot(np.append(move_force,1))
                
        self.p_tool[0]=self.p_tool[0]+new_vector[0]
        self.p_tool[1]=self.p_tool[1]+new_vector[1]
        self.p_tool[2]=self.p_tool[2]+new_vector[2]

    def set_object(self,ob):
        x = self.inv_rot_master.dot(np.append(ob,1)) 
        self.obj = np.array([x[0],x[1],x[2]])   
      
class mainframe():
    def __init__(self, names , num, gazebo_on):

        self.num = num
        self.names = names
        self.poses = [None]*self.num
        self.worldpose = [[0.15,0.15,0.15],[0.15,0.15,0.15],[0.15,0.15,0.15]]
        #print(self.worldpose)
        self.object_v = [None]*self.num
        # Create node
        if not rospy.get_node_uri():
            rospy.init_node('psm_controller')
            print('node initialized')
        else:
            rospy.loginfo(rospy.get_caller_id() + ' -> ROS already initialized')
        rospy.sleep(1)

      
        self.r = [None]*self.num
        self.state = [None]*self.num

        for i in range(self.num):         
            if (i==0):
                self.r[i] = master(self.names[i], gazebo_on)
            else:
                self.r[i] = slave(self.names[i], gazebo_on)
        
            #self.worldpose[i]=self.r[i].get_pose

        rospy.Subscriber('/psm/poses', gm.PoseStamped, self.handle_worldpose)
        rospy.Subscriber('/psm/cmd_vel', gm.Twist, self.handle_move)
        rospy.Subscriber('/psm/cmd_force', gm.Twist, self.handle_force)
        
          # Create publishers
        self.p = [None]*self.num
        for i in range(self.num):
            self.p[i] = rospy.Publisher('/' + self.names[i], JointState, queue_size=10)
            self.state[i] = rospy.Publisher('/psm_sense/'+self.names[i] +'/worldpose',gm.Pose,queue_size=10)
    
    def handle_worldpose(self,msg):
        i = self.names.index(msg.header.frame_id)
        self.worldpose[i] = np.array([msg.pose.position.x, msg.pose.position.y,msg.pose.position.z])

        #print(self.worldpose[i])
        #print("Got Message: " , i)
        self.make_object_v(i)

    def handle_move(self,msg):
        for i in range(self.num):
            self.r[i].move(msg)
            #print('this shit ',i,self.object_v[i])
            #print(self.r[i].obj)

            #For Video
            #self.r[i].move_abs(msg)

    def handle_force(self,msg):
        # for i in range(self.num-1):
        #     self.r[i+1].move_force(msg)
        for i in range(self.num):
            self.r[i].move_force(msg)

# def handle_object_pose(self,msg):
    #     for i in range(self.num-1):
    #         self.r[i+1].object_pose(msg)
    def joint_message_making(self,pose):
        scale = 100
        msg = gm.Pose()
        msg.position.x = pose[0]*scale
        msg.position.y = pose[1]*scale
        msg.position.z = pose[2]*scale

        return msg

    def make_centroid(self):
        #print(self.worldpose[1],self.worldpose[0])
        if (self.num == 2):
            self.object_v[0] = (np.array(self.worldpose[1])-np.array(self.worldpose[0]))/2
            self.object_v[1] = (np.array(self.worldpose[0])-np.array(self.worldpose[1]))/2
        
        else:    
            v1 = np.array(self.worldpose[1])-np.array(self.worldpose[0])
            v2 = np.array(self.worldpose[2])-np.array(self.worldpose[0])
            b = v2 - v1

            c_m = (v1+b/2)*2/3

            self.object_v[0] = c_m
            self.object_v[1] = c_m - v1
            self.object_v[2] = c_m - v2
        
        
    def make_object_v(self,x):
        if (self.num == 2):
            self.make_centroid()
            #This is for non centroid movement, has to be fixed
            # self.object_v[x] = self.worldpose[x]-self.worldpose[0]
            self.r[x].set_object(self.object_v[x])

            #print(self.r[x].obj)
        else:
            self.make_centroid()
            self.r[x].set_object(self.object_v[x])

    def run(self):
        for i in range(self.num):
            self.poses[i] = self.r[i].get_pose()
            self.r[i].send_transform()
            #print("send: " , i )
            #self.make_object_v(i)
            #print(self.worldpose[i])
            #print('got here')
            mess = self.joint_message_making(self.worldpose[i])
            self.state[i].publish(mess)

            data = self.r[i].inverse_kinematic(self.r[i].p_tool[0],self.r[i].p_tool[1],self.r[i].p_tool[2])
            for j in range(len(data)):
                msg = self.r[i].message_making(self.r[i].joint_names[j],data[j],j)
                self.p[i].publish(msg)

    

        



