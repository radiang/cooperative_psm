//
// Created by radian on 6/29/18.
//

#include "psm_coop/Cooperative.h"

int main(int argc, char **argv)
{
    string choose[2];
    ros::init(argc, argv, "PsmForceControl_node");


    choose[0] = "Cartesian";
    choose[1] = "Impedance";

    int num = 2;

    initializer psm[2];
    //psm[0]= psm1; psm[1] = psm2;


    psm[0].name = "PSM2";
    psm[0].type = "Master";
    psm[0].ctrl_type = choose[0];
    psm[0].track = false;
    psm[0].Rot.resize(3,3);
    psm[0].Rot << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;

    psm[0].Pos.resize(3);
    psm[0].Pos << 0, 0, 0;


    psm[1].name = "PSM1";
    psm[1].type = "Slave";
    psm[1].ctrl_type = choose[1];
    psm[1].track = true;

    psm[1].Rot.resize(3,3);
    //PSM1
    psm[1].Rot << 0.5430, -0.8397, 0.0082,
            0.8397, 0.5430, 0.0142,
            -0.0161, 0.0031, 0.9998;

    psm[1].Pos.resize(3);
    psm[1].Pos << 0.1353, 0.20372, -0.0308;


    //PSM2
/*    psm[1].Rot << 0.54285, 0.83962, -0.01636,
            -0.83973, 0.54299, -0.0007727,
            0.011334, 0.011856, .99991;

    psm[1].Pos.resize(3);
    psm[1].Pos << -0.24498, 0.0030182, 0.0026889;*/

    std::vector<initializer> m;

    m.push_back(psm[0]);
    m.push_back(psm[1]);


    Cooperative obj(m);
    ros::Duration(1).sleep();
    ros::Rate r(obj.Obj[0]->rate);


    //obj.CalcObject();
    while(ros::ok())
    {
        obj.Loopz();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}