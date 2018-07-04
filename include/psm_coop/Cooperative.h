//
// Created by radian on 6/29/18.
//

#ifndef PROJECT_COOPERATIVE_H
#define PROJECT_COOPERATIVE_H

#include "psm_coop/Psm.h"
#include "psm_coop/DataType.h"
#include <memory>
//#include <bits/shared_ptr.h>

class Cooperative {
public:
    long num;
    int count;

    Eigen::Vector3d offset;
    std::vector<Eigen::Vector3d> Pos, object;
    std::vector<std::shared_ptr<Psm>> Obj;
    std::shared_ptr<Psm> obj1, obj2;

    Cooperative(std::vector<initializer> &psm);

    void CalcObject();
    void Loopz();
    void CallbackMovez(const geometry_msgs::Twist &msg);
    void CallbackForcez(const geometry_msgs::Twist &msg);
};


#endif //PROJECT_COOPERATIVE_H
