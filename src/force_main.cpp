#include "psm_coop/Psm.h"
#include "psm_coop/DataType.h"


int main(int argc, char **argv)
{
    string choose[2];

    choose[0] = "Cartesian";
    choose[1] = "Impedance";

    ros::init(argc, argv, "PsmForceControl_node");

    int num = 1;
   // Options

    initializer psm[2];
    //psm[0]= psm1; psm[1] = psm2;

    psm[0].name = "PSM1";
    psm[0].type = "Master";
    psm[0].ctrl_type = choose[0];
    psm[0].Rot.resize(3,3);
    psm[0].Rot << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;

    psm[0].Pos.resize(3);
    psm[0].Pos << 0, 0, 0;

    psm[1].name = "PSM2";
    psm[1].type = "Slave";
    psm[1].ctrl_type = choose[0];

    // Input Calibration Values from Calibration Files
    psm[1].Rot.resize(3,3);
    psm[1].Rot << 0.54285, 0.83962, -0.01636,
            -0.83973, 0.54299, -0.0007727,
            0.011334, 0.011856, .99991;

    psm[1].Pos.resize(3);
    psm[1].Pos << -0.24498, 0.0030182, 0.0026889;

    std::shared_ptr<ros::NodeHandle> nhandle = std::make_shared<ros::NodeHandle>();
    std::shared_ptr<ros::NodeHandle> nhandle2 = std::make_shared<ros::NodeHandle>();

    Psm obj(nhandle, psm[0].name, psm[0].ctrl_type , psm[0].type,psm[0].track, psm[0].Rot, psm[0].Pos);
    Psm obj2(nhandle2, psm[1].name, psm[1].ctrl_type , psm[1].type,psm[1].track, psm[1].Rot, psm[1].Pos);

    ros::Rate r(obj.rate);
    ROS_INFO("It started");
    // int i, j;
    int count = 0;

    ros::spinOnce();
    ros::Duration(1).sleep();
    ros::spinOnce();

    obj.SetGainsInit();
    obj.SetDesiredInit();


    ROS_INFO_STREAM("v_int:" << obj.xe);

    if(num==2)
    {
        obj2.SetGainsInit();
        obj2.SetDesiredInit();
    }

    ros::Duration(1).sleep();

    while(ros::ok())
    {
        obj.Loop();
        if(num==2) {
            obj2.Loop();
        }
        ros::spinOnce();

        //drop=0;

        r.sleep();
    }
//ros::spin();
return 0;
}
