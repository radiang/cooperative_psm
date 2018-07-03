#ifndef CInterpolateH
#define CInterpolateH

#include <iostream>
#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include<vector>
#include <stdlib.h>
#include <sstream>


#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>


#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "std_msgs/String.h"

#include "psm_coop/DataType.h"

using namespace std;




traject interpolate(traject des);

#endif