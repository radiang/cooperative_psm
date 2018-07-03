//
// Created by radian on 6/29/18.
//
#include "psm_coop/Cooperative.h"

Cooperative::Cooperative(std::vector<initializer> &psm)
{
    //force_sub = n.subscribe("/psm/cmd_force", 10, &Cooperative::CallbackForce, this);
    //setpos_sub2 = n.subscribe("/psm/cmd_vel", 10, &Cooperative::CallbackMove, this);

    num = psm.size();

    std::cout<< psm[0].name;

    ros::NodeHandle nandle;
    std::shared

    Psm obj1(&nandle, psm[0].name, psm[0].ctrl_type , psm[0].type, psm[0].Rot, psm[0].Pos);
    Psm obj2(&nandle, psm[1].name, psm[1].ctrl_type , psm[1].type, psm[1].Rot, psm[1].Pos);

    ros::spinOnce();
    ros::Duration(1).sleep();
    ros::spinOnce();

    Obj.push_back(obj1);
    Obj.push_back(obj2);

    //Obj[0].xe <<2,1,3;
    //Obj[1].xe <<0,0,0;

    offset = Obj[0].pos-Obj[1].pos;

    Pos.push_back(Obj[0].xe);
    Pos.push_back(offset + Obj[1].xe);

    //ROS_INFO_STREAM("Pos:" << Pos[1]);

    ros::Rate r(Obj[0].rate);


    object.push_back(Pos[1]-Pos[0]); //Initiate Object
    object.push_back(-object[0]);   // Initiate Object


    //Set Gains and Initialize Controller

    for(int i;i<num;i++)
    {
        Obj[i].SetGainsInit();
        Obj[i].SetDesiredInit();
    }

    ros::Duration(1).sleep();

}

void Cooperative::CalcObject()
{
    object[0] = Pos[1]-Pos[0];
    object[1] = -object[0];

    //std::cout << object[0];
    for(int i;i<num;i++)
    {
        Obj[i].SetObject(object[i]);
    }
}

void Cooperative::Loopz()
{
    //this->CalcObject();

    for(int i;i<num;i++)
    {
        //int x=0;
        Obj[i].Loop();

    }
    ros::spinOnce();
}

void Cooperative::CallbackMovez(const geometry_msgs::Twist &msg)
{
    for(int i;i<num;i++)
    {
        Obj[i].CallbackSetPositionIncrement(msg);
    }
}

void Cooperative::CallbackForcez(const geometry_msgs::Twist &msg)
{
    for(int i;i<num;i++)
    {
        Obj[i].CallbackForce(msg);
    }
}