#ifndef CInterpolateH
#define CInterpolateH

#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include<vector>
#include <stdlib.h>
#include <boost/bind.hpp>
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


using namespace std;

struct traject {
    Eigen::VectorXd x;
    Eigen::VectorXd v;
    Eigen::VectorXd a;
    double ts;
    double tf;
    bool check;
    bool check2;
    Eigen::VectorXd qd;
    };


traject interpolate(traject des);

#endif