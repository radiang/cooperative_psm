#ifndef CPsmH
#define CPsmH

#include "psm_coop/force_control.h"

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



    Psm(std::shared_ptr<ros::NodeHandle> n, const string nam,const string ctrl_type, const string typ, bool trak, const Eigen::MatrixXd Rotz, const Eigen::VectorXd Posz);

   void CallbackSetPositionIncrement(const geometry_msgs::Twist &msg);
   void CallbackSetPosition(const geometry_msgs::Twist &msg);
   void CallbackSetForceIncrement(const geometry_msgs::Twist &msg);
   void SetObject(const Eigen::Vector3d &set);
   Eigen::Vector3d GetPose();
   void ForceLoop();
   void ForceSet();

};




#endif