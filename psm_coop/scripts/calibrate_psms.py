#!/usr/bin/env python

#  Finds transformation matrix for 3 optical markers attached to the mount to calibrate
#  dvrk-psm-force-feedback device
# gives transformation_matrix.txt and transformation_matrix.npz files as an output

from psm_coop import tracker
#from scipy.cluster.vq import kmeans, whiten
import numpy as np
import dvrk
import rospy

if __name__ == '__main__':

    p = [None]*2

    p1 = dvrk.psm('PSM2')
    p2 = dvrk.psm('PSM1')

    limit_q1 = 0.75
    limit_q2 = 0.25
    limit_q3 = 0.05

    p1.move_joint_some(np.array([-limit_q1, -limit_q2, limit_q3]), np.array([0, 1, 2]))
    p2.move_joint_some(np.array([-limit_q1, -limit_q2, limit_q3]), np.array([0, 1, 2]))

    rate = 200
    tf = 2

    r = rospy.Rate(rate)

    z = 10
    z2 = rate *tf

    
    q1 = np.linspace(-limit_q1, limit_q1, num=z)
    q2 = np.linspace(-limit_q2, limit_q2, num=z2)
    i = 0
    j = 0


    print("I'm here")
        #track = tracker.RotationTracker()

    while i < z and not rospy.is_shutdown():
        j = 0
        p1.move_joint_some(np.array([q1[i], q2[j], limit_q3]), np.array([0, 1, 2]))
        p2.move_joint_some(np.array([q1[i], q2[j], limit_q3]), np.array([0, 1, 2]))
            

        while j < z2 and not rospy.is_shutdown():
            p1.move_joint_some(np.array([q1[i], q2[j], limit_q3]), np.array([0, 1, 2]), False)
            p2.move_joint_some(np.array([q1[i], q2[j], limit_q3]), np.array([0, 1, 2]), False)
            
            r.sleep()
            j = j + 1

        i = i + 1

    
    p2.move_joint_some(np.array([-0, -0, limit_q3]), np.array([0, 1, 2]))
    p1.move_joint_some(np.array([-0, -0, limit_q3]), np.array([0, 1, 2]))
    x = 0

