//
// Created by radian on 7/22/18.
//

#ifndef PROJECT_PSM_SUB_H
#define PROJECT_PSM_SUB_H

#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;

class PsmSub{
private:
    ros::Subscriber Sub_xd, Sub_xe, Sub_xf;
    ros::Publisher Pub_xd;

    string name;
    shared_ptr<ros::NodeHandle> nhandle;

    geometry_msgs::Pose xd_msg;

public:

    Eigen::Vector3d xe, xd, xf, xd_init;

    PsmSub(shared_ptr<ros::NodeHandle> n, const string nam);

    void CallbackXe(const geometry_msgs::Pose &msg);
    void CallbackXd(const geometry_msgs::Pose &msg);
    void CallbackXf(const geometry_msgs::Pose &msg);
    void PublishXd();
    void GetInit();
};

#endif //PROJECT_PSM_SUB_H
