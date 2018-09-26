//
// Created by radian on 6/29/18.
//
#include "psm_coop/Cooperative.h"

Cooperative::Cooperative(std::vector<initializer> &psm)
{
    count = 0;

    num = psm.size();

    std::cout<< psm[0].name;

    std::shared_ptr<ros::NodeHandle> nhandle = std::make_shared<ros::NodeHandle>();
    std::shared_ptr<ros::NodeHandle> nhandle2 = std::make_shared<ros::NodeHandle>();


    obj1 = std::make_shared<Psm>(nhandle, psm[0].name, psm[0].ctrl_type , psm[0].type, psm[0].track, psm[0].Rot, psm[0].Pos);
    obj2 = std::make_shared<Psm>(nhandle2, psm[1].name, psm[1].ctrl_type , psm[1].type, psm[1].track, psm[1].Rot, psm[1].Pos);


    ros::spinOnce();
    ros::Duration(1).sleep();
    ros::spinOnce();

    Obj.push_back(obj1);
    Obj.push_back(obj2);

    offset = Obj[1]->pos-Obj[0]->pos;

    Pos.push_back(Obj[0]->GetPose());
    Pos.push_back(offset + Obj[1]->rot*Obj[1]->GetPose());



    object.push_back(Pos[1]-Pos[0]); //Initiate Object
    object.push_back(-object[0]);   // Initiate Object

    //Set Gains and Initialize Controller
    for(int i;i<num;i++)
    {
        Obj[i]->SetGainsInit();
        Obj[i]->SetDesiredInit();
    }

    ros::Duration(1).sleep();

}

void Cooperative::CalcObject()
{
    Pos[0] = Obj[0]->GetPose();
    Pos[1] = offset + Obj[1]->rot*Obj[1]->GetPose();

    object[0] = Pos[1]-Pos[0];
    object[1] = Pos[0]-Pos[1];

    for(int i;i<num;i++)
    {
        Obj[i]->SetObject(object[i]);
    }
}

void Cooperative::Loopz() {
    this->CalcObject();

    for (int i; i < num; i++)
    {
        Obj[i]->Loop();

/*        if(count%4==0)
        {
            Obj[i]->ForceLoop();
            count = 0;
        }*/

        Obj[i]->ForceLoop();
        Obj[i]->DataCollect();
        count = 0;

    }
    count = count + 1;
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
        Obj[i]->CallbackSetForceIncrement(msg);
    }
}

