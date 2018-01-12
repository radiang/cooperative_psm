#!/usr/bin/env python  
import sys
import rospy
from gazebo_msgs.srv import GetLinkState

def get_link_state(x,y):
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        get = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        resp1 = get(x,y)
        print resp1.link_state.link_name
        print resp1.link_state.pose.position

        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    rospy.init_node('service_caller')
    r = rospy.Rate(800)

    if len(sys.argv) == 3:
        x = str(sys.argv[1])
        y = str(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    while not rospy.is_shutdown():
        get_link_state(x,y)



