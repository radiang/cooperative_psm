#ifndef CPsmH
#define CPsmH

#include "psm_coop/force_control.h"
#include "psm_coop/csv_writer.h"

class Psm: public PsmForceControl
{

public:
    bool track;
    string type;
    string ctrl_typ;
    Eigen::MatrixXd rot;
    Eigen::Vector3d pos, object;
    Eigen::Vector3d xs;
    Eigen::Vector3d temp_x;

    double Kf, mag, delta_mag;

    double sampling_freq = 200;
    bool sampling_start = false;
    double sampling_start_time = 20;//start sampling after 10s
    int sampling_num = (int)sampling_freq*30; //sample for 10s
    int sampling_cnt = 0;
    int joint_state_cnt = 0;

    std::vector<Eigen::Vector3d> vector_xd, vector_xe, vector_xf;

    Psm(std::shared_ptr<ros::NodeHandle> n, const string nam,const string ctrl_type, const string typ, bool trak, const Eigen::MatrixXd Rotz, const Eigen::VectorXd Posz);

   void CallbackSetPositionIncrement(const geometry_msgs::Twist &msg);
   void CallbackSetPosition(const geometry_msgs::Twist &msg);
   void CallbackSetForceIncrement(const geometry_msgs::Twist &msg);
   void SetObject(const Eigen::Vector3d &set);
   Eigen::Vector3d GetPose();
   void ForceLoop();
   void ForceSet();
   void Write(string file_name, vector<Eigen::Vector3d> *v_j);
   void DataCollect();

};




#endif