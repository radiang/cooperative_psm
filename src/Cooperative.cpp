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

    std::shared_ptr<ros::NodeHandle> nhandle = std::make_shared<ros::NodeHandle>();


    obj1 = std::make_shared<Psm>(nhandle, psm[0].name, psm[0].ctrl_type , psm[0].type, psm[0].Rot, psm[0].Pos);
    obj2 = std::make_shared<Psm>(nhandle, psm[1].name, psm[1].ctrl_type , psm[1].type, psm[1].Rot, psm[1].Pos);


    ros::spinOnce();
    ros::Duration(1).sleep();
    ros::spinOnce();

    Obj.push_back(obj1);
    Obj.push_back(obj2);

    offset = Obj[1]->pos-Obj[0]->pos;

    Pos.push_back(Obj[0]->GetPose());
    Pos.push_back(offset + Obj[1]->rot*Obj[1]->GetPose());

    //ROS_INFO_STREAM("FAULT:" << Obj[0]->xe);

    //ros::Rate r(2000);

    object.push_back(Pos[1]-Pos[0]); //Initiate Object
    object.push_back(-object[0]);   // Initiate Object

    //Set Gains and Initialize Controller

    for(int i;i<num;i++)
    {
        Obj[i]->SetGainsInit();
        Obj[i]->SetDesiredInit();
    }

    ros::Duration(1).sleep();
    ROS_INFO_STREAM("FAULT:" << Obj.size());

}

void Cooperative::CalcObject()
{
    Pos[0] = Obj[0]->GetPose();
    Pos[1] = offset + Obj[1]->rot*Obj[1]->GetPose();

    object[0] = Pos[1]-Pos[0];
    object[1] = Pos[0]-Pos[1];

    //std::cout << object[0];
    for(int i;i<num;i++)
    {
        Obj[i]->SetObject(object[i]);
    }
}

void Cooperative::Loopz() {
    this->CalcObject();

    for (int i; i < num; i++)
    {
        //int x=0;
        Obj[i]->Loop();
    }
    //ros::spinOnce();


}

void Cooperative::CallbackMovez(const geometry_msgs::Twist &msg)
{
    for(int i;i<num;i++)
    {
        Obj[i]->CallbackSetPositionIncrement(msg);
    }
}

void Cooperative::CallbackForcez(const geometry_msgs::Twist &msg)
{
    for(int i;i<num;i++)
    {
        Obj[i]->CallbackForce(msg);
    }
}