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

    int num = 1;

    initializer psm[2];
    //psm[0]= psm1; psm[1] = psm2;


    psm[0].name = "PSM1";
    psm[0].type = "Master";
    psm[0].ctrl_type = choose[0];
    psm[0].track = false;
    psm[0].Rot.resize(3,3);
    psm[0].Rot << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;

    psm[0].Pos.resize(3);
    psm[0].Pos << 0, 0, 0;

    psm[1].name = "PSM2";
    psm[1].type = "Slave";
    psm[1].ctrl_type = choose[0];
    psm[1].track = false;
    psm[1].Rot.resize(3,3);
    psm[1].Rot << 0.54285, 0.83962, -0.01636,
            -0.83973, 0.54299, -0.0007727,
            0.011334, 0.011856, .99991;

    psm[1].Pos.resize(3);
    psm[1].Pos << -0.24498, 0.0030182, 0.0026889;

    std::vector<initializer> m;

    m.push_back(psm[0]);
    m.push_back(psm[1]);


    Cooperative obj(m);
    ros::Duration(1).sleep();
    ros::Rate r(2000);


    //obj.CalcObject();
    while(ros::ok())
    {
        obj.Loopz();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}