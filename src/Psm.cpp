#include "psm_coop/Psm.h"


Psm::Psm(ros::NodeHandle n,const string nam, const string typ, const Eigen::MatrixXd Rotz, const Eigen::VectorXd Posz):PsmForceControl(n, nam), type(typ)
{
    Rot.resize(3,3);
    Pos.resize(3);

    Rot = Rotz;
    Pos = Posz;
}

void Psm::CallbackSetPositionIncrement(const geometry_msgs::Twist &msg)
{

    double arr[3];

    arr[0] = msg.linear.x;
    arr[1] = msg.linear.y;
    arr[2] = msg.linear.z;

    if (type == "Master")
    {
        xd(0)= msg.linear.x + xd(0);
        xd(1)= msg.linear.y + xd(1);
        xd(2)= msg.linear.z + xd(2);
    }

    else if (type == "Slave")
    {
        xd(0)= msg.linear.x + xd(0);
        xd(1)= msg.linear.y + xd(1);
        xd(2)= msg.linear.z + xd(2);
    }

    for (int i=0;i<3;i++)
    {
        // Impedance Controller
        q_traj[i].qd << xe(i), 0, 0, xd(i),0,0;

        // Computed Torque controller
        //q_traj[i].qd << q(i), 0, 0, xd(i),0,0;


        if (arr[i] != 0)
        {
            q_traj[i]=interpolate(q_traj[i]);
        }
    }

    //ROS_INFO_STREAM("v_int:" << q_traj[0].check);

    t0 = ros::Time::now().toSec();
    t = 0;
    interp = true;




}