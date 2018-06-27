#ifndef CPsmForceControlH
#define CPsmForceControlH


#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <boost/bind.hpp>
#include <sstream>
#include <queue>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "std_msgs/String.h"

#include "psm_coop/Interpolate.h"

using namespace std;

class PsmForceControl{
private:
  //std::vector<ros::Publisher> ecmPub, psm1Pub, psm2Pub, psm3Pub;
    ros::Subscriber jacobian_sub, joint_sub, cartesian_sub, force_sub, setforce_sub, setpos_sub, setpos_sub2;
    ros::Publisher plot_x, plot_y, plot_z, joint_pub, pose_pub, desplot_x, desplot_y, desplot_z;

  //std::vector<ros::Publisher> cartPub;
public:
    PsmForceControl(ros::NodeHandle n, const string nam, const string ctrl_type);

    double rate;
    int t;
    double t0;
    double tf;
    int index;
    string name;
    string ctrl;


    int filter_n;
    int drop;
    int drop_p;
    int dof;
    int cart_dof;

    bool interp;

    double sum[6];
    std::deque<double> myq[6];
    std::deque<double> que1, que2, que3, que4, que5, que6;

    traject q_traj[3], q1_traj, q2_traj, q3_traj;

    Eigen::VectorXd q, qd, eff,xe, ve, fd, he, xf, xd, vd,ad, y, u, x0, q0, N, x_int, v_int, a_int, G, Fr, deadband;
    Eigen::VectorXd joint_act, joint_des, orient_cart;

    Eigen::VectorXd wrist_u, wrist_eq, wrist_eqd, wrist_kp, wrist_kd;
    Eigen::MatrixXd Ja, JaM, Jd, Jmin , C;
    Eigen::MatrixXd M, Mt, Kp, Kd, Cp, Ci;
    Eigen::MatrixXd data_trans;

    std_msgs::Float64 mq0, mq1, mq2, dq0, dq1, dq2;
    sensor_msgs::JointState joint_msg, msg2;

    geometry_msgs::Pose pose_msg;


  void SetGainsInit();
  void SetDesiredInit();

  void CallbackJacobian(const std_msgs::Float64MultiArray &msg);
  void CallbackJoint(const sensor_msgs::JointState &msg);
  void CallbackCartesian(const geometry_msgs::PoseStamped &msg);
  void CallbackForce(const geometry_msgs::Twist &msg);
  void CallbackSetForce(const geometry_msgs::Pose &msg);
  void CallbackSetPositionIncrement(const geometry_msgs::Twist &msg);
  void CallbackSetPosition(const geometry_msgs::Twist &msg);

  void CalcJaM(const Eigen::VectorXd &q,const Eigen::VectorXd &qd);
  void CalcDiffJacobian(const Eigen::VectorXd &q, const Eigen::VectorXd  &qd);

  void CalcN(const Eigen::VectorXd &q,const Eigen::VectorXd &qd);
  void CalcFr(const Eigen::VectorXd &q, const Eigen::VectorXd &qd);
  void CalcC(const Eigen::VectorXd &q, const Eigen::VectorXd &qd);
  void CalcG(const Eigen::VectorXd &q);
  void CalcM(const Eigen::VectorXd &q);
  void CalcU();
  void WristPID();
  Eigen::VectorXd InverseKinematic(const Eigen::VectorXd &fed);
  void output();
  void Loop();

};


#endif