//
// Created by radian on 7/22/18.
//

#include<psm_coop/psm_sub.h>

PsmSub::PsmSub(shared_ptr<ros::NodeHandle> n, const string nam){
    nhandle = n;
    name = nam;

    Sub_xe=nhandle->subscribe("/psm_sense/" + name + "/actual_pose", 10, &PsmSub::CallbackXe, this);
    Sub_xd=nhandle->subscribe("/psm_sense/" + name + "/desired_pose_(TemporarilyNotMakeWork)", 10, &PsmSub::CallbackXd, this);
    Sub_xf=nhandle->subscribe("/psm_sense/" + name + "/actual_pose", 10, &PsmSub::CallbackXf, this);

    Pub_xd = nhandle->advertise<geometry_msgs::Pose>("/psm/" +name + "/cmd_pos",1);
}
void PsmSub:: CallbackXe(const geometry_msgs::Pose &msg){
    xe(0) = msg.position.x;
    xe(1) = msg.position.y;
    xe(2) = msg.position.z;
}

void PsmSub:: CallbackXd(const geometry_msgs::Pose &msg){
    xd(0) = msg.position.x;
    xd(1) = msg.position.y;
    xd(2) = msg.position.z;
}

void PsmSub:: CallbackXf(const geometry_msgs::Pose &msg){
    xf(0) = msg.position.x;
    xf(1) = msg.position.y;
    xf(2) = msg.position.z;
}

void PsmSub::PublishXd() {

    xd_msg.position.x = xd(0);
    xd_msg.position.x = xd(1);
    xd_msg.position.x = xd(2);

    Pub_xd.publish(xd_msg);
}

void PsmSub::GetInit() {
    xd_init = xe;
}