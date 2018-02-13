#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('PSMs_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((.5 , 0, 0.2),
                         (0.0, 0.0, 0.0, 1),
                         rospy.Time.now(), 
                         "PSM1_w", "world"
                         )
        br.sendTransform((0, .5, 0.3),
                         (0, 0.0, 0.0, 1),
                         rospy.Time.now(),
                         "PSM1_1",
                         "PSM1_w")
	
	br.sendTransform((-1, 0, 0),       tf.transformations.quaternion_from_euler(0, 0, -1.57/3),
                         rospy.Time.now(),
                         "PSM2_w","PSM1_w")
	br.sendTransform((0, .5, 0.3),
                         (0, 0, 0, 1),
                         rospy.Time.now(),
                         "PSM2","PSM2_w")        
rate.sleep()
