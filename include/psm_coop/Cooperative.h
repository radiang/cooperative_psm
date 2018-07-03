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
    //Psm p1, p2;

    Cooperative(std::vector<initializer> &psm);

    void CalcObject();
    void Loopz();
    void CallbackMovez(const geometry_msgs::Twist &msg);
    void CallbackForcez(const geometry_msgs::Twist &msg);
};


#endif //PROJECT_COOPERATIVE_H
