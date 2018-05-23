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


traject interpolate(traject des)
{

    double t0 = 0;
    double tf = des.tf;

    Eigen::VectorXd coef;
    Eigen::MatrixXd T_mat;
    coef.resize(6);
    T_mat.resize(6,6);

    T_mat << 1, t0, pow(t0,2),     pow(t0,3),      pow(t0,4),      pow(t0,5),
            0,  1,  2*t0, 3*pow(t0,2),  4*pow(t0,3),  5*pow(t0,4),
            0,  0,     2,      6*t0, 12*pow(t0,2), 20*pow(t0,3),
            1, tf, pow(tf,2),     pow(tf,3),      pow(tf,4),   pow(tf,5),
            0,  1,  2*tf, 3*pow(tf,2),  4*pow(tf,3),  5*pow(tf,4),
            0,  0,     2,      6*tf, 12*pow(tf,2), 20*pow(tf,3);

    coef = T_mat.inverse()*des.qd;



    int period = des.tf/des.ts;

    //cout<<period;

    des.x.resize(period);
    des.v.resize(period);
    des.a.resize(period);

    double t;
    double x;
    double v;
    double a;

    for (int i=0;i<period;i++) {
        t = t0 + des.ts * (i + 1);
        x = coef(0) + coef(1) * t + coef(2) * pow(t, 2) + coef(3) * pow(t, 3) + coef(4) * pow(t, 4) +
            coef(5) * pow(t, 5);
        v = coef(1) + 2 * coef(2) * t + 3 * coef(3) * pow(t, 2) + 4 * coef(4) * pow(t, 3) + 5 * coef(5) * pow(t, 4);
        a = 2 * coef(2) * t + 6 * coef(3) * t + 12 * coef(4) * pow(t, 2) + 20 * coef(5) * pow(t, 3);

        //std::cout<<x;
        if (des.check2 == false) {
            des.x(i) = x;
            des.v(i) = v;
            des.a(i) = a;
        }
        else {
            des.x(i) = des.qd(0);
            des.v(i) = des.qd(1);
            des.a(i) = des.qd(2);
        }
    }
    //std::cout<<des.x(10);

    des.check = true;


    return des;
}