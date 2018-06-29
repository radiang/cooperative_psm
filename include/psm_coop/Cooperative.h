//
// Created by radian on 6/29/18.
//

#ifndef PROJECT_COOPERATIVE_H
#define PROJECT_COOPERATIVE_H

#include "psm_coop/Psm.h"
#include "psm_coop/DataType.h"

class Cooperative {
public:
    long num;
    Eigen::Vector3d offset;
    std::vector<Eigen::Vector3d> Pos, object;
    std::vector<Psm> Obj;

    ros::Subscriber force_sub, setforce_sub, setpos_sub, setpos_sub2;
    Cooperative(std::vector<initializer> &psm, ros::NodeHandle n);

    void CalcObject();
    void CallbackMove(const geometry_msgs::Twist &msg);
    void CallbackForce(const geometry_msgs::Twist &msg);
};


#endif //PROJECT_COOPERATIVE_H
