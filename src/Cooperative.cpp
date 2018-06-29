//
// Created by radian on 6/29/18.
//
#include "psm_coop/Cooperative.h"

Cooperative::Cooperative(std::vector<initializer> &psm, ros::NodeHandle n)
{
    force_sub = n.subscribe("/psm/cmd_force", 10, &Cooperative::CallbackForce, this);
    setpos_sub2 = n.subscribe("/psm/cmd_vel", 10, &Cooperative::CallbackMove, this);

    num = psm.size();

    std::cout<< num;

    Psm obj1(psm[0].n, psm[0].name, psm[0].ctrl_type , psm[0].type, psm[0].Rot, psm[0].Pos);
    Psm obj2(psm[1].n, psm[1].name, psm[1].ctrl_type , psm[1].type, psm[1].Rot, psm[1].Pos);

    Obj.push_back(obj1);
    Obj.push_back(obj2);

    Obj[0].xe <<2,1,3;
    Obj[1].xe <<0,0,0;

    offset = Obj[0].pos-Obj[1].pos;

    Pos.push_back(Obj[0].xe);
    Pos.push_back(offset-Obj[1].xe);


    object.push_back(Pos[0]-Pos[1]);
    object.push_back(-object[0]);
}

void Cooperative::CalcObject()
{
    object[1] = Pos[0]-Pos[1];
    object[0] = -object[1];
    std::cout<<object[0];
}

void Cooperative::CallbackMove(const geometry_msgs::Twist &msg)
{
    for(int i;i<num;i++)
    {
        Obj[i].CallbackSetPositionIncrement(msg);
    }
}

void Cooperative::CallbackForce(const geometry_msgs::Twist &msg)
{
    for(int i;i<num;i++)
    {
        Obj[i].CallbackForce(msg);
    }
}