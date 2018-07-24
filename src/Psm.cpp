#include "psm_coop/Psm.h"


Psm::Psm(std::shared_ptr<ros::NodeHandle> n,const string nam, const string ctrl_type, const string typ, bool trak, const Eigen::MatrixXd Rotz, const Eigen::VectorXd Posz):PsmForceControl(n, nam, ctrl_type), type(typ)
{

    rot.resize(3,3);

    rot = Rotz;
    pos = Posz;

    temp_x << 0.0, 0.0, 0.0;

    //Force Stuff
    Kf = .00001; //  [ m/N ];
    mag = 0;
    delta_mag = 0;
    force_deadband = 0.3;

    //type = typ;
    //ctrl_typ = ctrl_type;
    track = trak;

    //Init record data
    vector_xd.resize(sampling_num);
    vector_xf.resize(sampling_num);
    vector_xe.resize(sampling_num);
}

void Psm::CallbackSetPosition(const geometry_msgs::Twist &msg) {

    Eigen::Vector3d error;
    double inc_limit = 0.002;

    error(0) = xd(0) - msg.linear.x;
    error(1) = xd(1) - msg.linear.y;
    error(2) = xd(2) - msg.linear.z;

    if(error.norm()<inc_limit) {

        if (type == "Master") {

            xd(0) = msg.linear.x;
            xd(1) = msg.linear.y;
            xd(2) = msg.linear.z;

        } else if (type == "Slave") {

            xs(0) = msg.linear.x;
            xs(1) = msg.linear.y;
            xs(2) = msg.linear.z;

            xd = rot.inverse() * xs;
        }
        else
        {
            ROS_INFO_STREAM("Increment too large");
        }
    }
}
void Psm::CallbackSetPositionIncrement(const geometry_msgs::Twist &msg)
{

    if (type == "Master")
    {
        arr(0) = msg.linear.x;
        arr(1) = msg.linear.y;
        arr(2) = msg.linear.z;

        xd = arr + xd;

       /* xd(0)= msg.linear.x + xd(0);
        xd(1)= msg.linear.y + xd(1);
        xd(2)= msg.linear.z + xd(2);*/
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


        if (arr(i) != 0)
        {
            q_traj[i]=interpolate(q_traj[i]);
        }
    }

    //ROS_INFO_STREAM("v_int:" << q_traj[0].check);

    t0 = ros::Time::now().toSec();
    t = 0;
    interp = true;

}

void Psm::CallbackSetForceIncrement(const geometry_msgs::Twist &msg)
{   Eigen::Vector3d x;
    //ROS_INFO_STREAM(type<<track);
    if (type == "Slave") {
        if (track == false) {

            Eigen::Vector3d x;
            x = object / object.norm() * -msg.linear.x;

            xf = xf + data_trans * x;
        }
        else {
            force_set = force_set + 100*msg.linear.x;
        }
    }

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

    //ROS_INFO_STREAM(name <<" object" << object);
}

Eigen::Vector3d Psm::GetPose()
{
    return data_trans.inverse()*xe;
}

void Psm::ForceLoop()
{
    //ROS_INFO_STREAM(name <<"yeahh");
    force_error = force_set-force_magnitude;
    if(track == true)
    {  //ROS_INFO_STREAM(name <<"ugh yeahh");
        if(std::abs(force_error)>force_deadband)
        {
            //temp_x = object / object.norm() * -Kf*(force_error);
            delta_mag =  Kf*(force_error);
        }
      /*  else if (force_error<-force_deadband)
        {
            temp_x = object / object.norm() * force_increment;
        }*/
        else {
            temp_x << 0.0, 0.0, 0.0;
            delta_mag = 0;
        }

        mag = mag + delta_mag;

        xf = data_trans * -object / object.norm() * mag;
    }
}

void Psm::ForceSet()
{
   // he = data_trans * object/object.norm()*force_magnitude;
}

void Psm::Write(string file_name, vector<Eigen::Vector3d> *v_j)
{
    CSVWriter writer(file_name);
    vector<double> v3 = {0, 0, 0};

    for(auto it = v_j->begin(); it != v_j->end(); ++it)
    {
        v3[0] = it->data()[0];
        v3[1] = it->data()[1];
        v3[2] = it->data()[2];
        writer.addDatainRow(v3.begin(), v3.end());
    }
}

void Psm::DataCollect() {
    if(interp=true){
        sampling_start = true;
    }
    if(sampling_start)
    {
        int sampling_cnt_base = round(rate/sampling_freq);
        if(joint_state_cnt%sampling_cnt_base == 0)
        {
            if(sampling_cnt < sampling_num)
            {
                vector_xd[sampling_cnt] = xd;
                vector_xe[sampling_cnt] = xe;
                vector_xf[sampling_cnt] = xf;
            }
            else if(sampling_cnt == sampling_num)
            {
                //write data into files
                string filename = name + "_xd.csv";
                Write(filename, &vector_xd);

                filename = name + "_xe.csv";
                Write(filename, &vector_xe);

                filename = name + "_xf.csv";
                Write(filename, &vector_xf);
            }
            sampling_cnt++;
        }
        joint_state_cnt++;
    }

}