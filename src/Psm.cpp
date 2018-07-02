#include "psm_coop/Psm.h"


Psm::Psm(ros::NodeHandle n,const string nam, const string ctrl_type, const string typ, const Eigen::MatrixXd Rotz, const Eigen::VectorXd Posz):PsmForceControl(n, nam, ctrl_type), type(typ)
{

    rot.resize(3,3);

    rot = Rotz;
    pos = Posz;
}

void Psm::CallbackSetPositionIncrement(const geometry_msgs::Twist &msg)
{

    Eigen::Vector3d arr;

    //ROS_INFO_STREAM("M: this is Yan");

    if (type == "Master")
    {
        arr(0) = msg.linear.x;
        arr(1) = msg.linear.y;
        arr(2) = msg.linear.z;

        xd(0)= msg.linear.x + xd(0);
        xd(1)= msg.linear.y + xd(1);
        xd(2)= msg.linear.z + xd(2);
    }

    else if (type == "Slave")
    {

        xs(0)= msg.linear.x;
        xs(1)= msg.linear.y;
        xs(2)= msg.linear.z;

        arr = rot.inverse()*xs;

        xd = xd + arr;

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

void Psm::CallbackForce(const geometry_msgs::Twist &msg)
{
    Eigen::Vector3d x;
    x = object*msg.linear.x;

    xd= xd + x;

}

void Psm::SetObject(const Eigen::Vector3d &set)

{
    if (type == "Master")
    {
       object = set;
    }

    else if (type == "Slave")
    {
        object = rot.inverse()*set;
    }

    ROS_INFO_STREAM(name <<" object" << object);
}