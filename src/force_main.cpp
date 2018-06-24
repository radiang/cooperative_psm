#include "psm_coop/Psm.h"


int main(int argc, char **argv)
{
   // Options
    string name = "PSM1";
    string master = "Master";

    Eigen::MatrixXd Rotation;
    Eigen::VectorXd Position;

    Rotation.resize(3,3);
    Rotation << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;

    Position.resize(3);
    Position << 0, 0, 0;

    ros::init(argc, argv, "PsmForceControl_node");
    ros::NodeHandle n;
    ROS_INFO("It started");

    Psm obj(n, name, master, Rotation, Position);
    ros::Rate r(obj.rate);

    // int i, j;
    int count = 0;
    ros::spinOnce();
    ros::Duration(1).sleep();
    ros::spinOnce();

    obj.SetGainsInit();
    obj.SetDesiredInit();

    while(ros::ok())
    {
        obj.Loop();
        ros::spinOnce();

        //drop=0;

        r.sleep();
    }
//ros::spin();

}
