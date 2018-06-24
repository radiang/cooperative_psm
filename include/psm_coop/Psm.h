#ifndef CPsmH
#define CPsmH

#include "psm_coop/force_control.h"

class Psm: public PsmForceControl
{

public:
    string type;
    Eigen::MatrixXd Rot;
    Eigen::VectorXd Pos;

    Psm(ros::NodeHandle n, const string nam, const string typ, const Eigen::MatrixXd Rotz, const Eigen::VectorXd Posz);

   void CallbackSetPositionIncrement(const geometry_msgs::Twist &msg);
   void CallbackSetForceIncrement(const geometry_msgs)

};



#endif