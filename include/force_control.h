#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <boost/bind.hpp>
#include <sstream>


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

#include "Interpolate.h"

class PsmForceControl{
private:
  //std::vector<ros::Publisher> ecmPub, psm1Pub, psm2Pub, psm3Pub;
  ros::Subscriber jacobian_sub, joint_sub, cartesian_sub, force_sub, setforce_sub, setpos_sub, setpos_sub2;
  ros::Publisher plot_x, plot_y, plot_z, joint_pub;

  //std::vector<ros::Publisher> cartPub;
public:
    double rate;
    int t;
    double t0;
    double tf;

    traject q1_traj,q2_traj,q3_traj;

  Eigen::VectorXd q, qd, eff,xe, ve, fd, he, xf, xd, vd,ad, y, u, x0, q0, N, x_int, v_int, a_int, G, Fr;
  Eigen::MatrixXd Ja, JaM, Jd, Jmin , C;
  Eigen::MatrixXd M, Mt, Kp, Kd, Cp, Ci;

  //Eigen::Matrix4d Ja;

  std_msgs::Float64 msg1;
  sensor_msgs::JointState joint_msg, msg2;

  PsmForceControl(ros::NodeHandle n){
    plot_y=n.advertise<std_msgs::Float64>("/qd",10);
    plot_x=n.advertise<sensor_msgs::JointState>("/qd1",10);
    joint_pub=n.advertise<sensor_msgs::JointState>("/dvrk/PSM1/set_effort_joint",10);

    jacobian_sub=n.subscribe("/dvrk/PSM1/jacobian_body", 100, &PsmForceControl::callback_jacobian,this);
    joint_sub=n.subscribe("/dvrk/PSM1/state_joint_current", 100, &PsmForceControl::callback_joint,this);
    cartesian_sub=n.subscribe("/dvrk/PSM1/position_cartesian_current", 100, &PsmForceControl::callback_cartesian,this);

    force_sub=n.subscribe("/psm_sense/PSM2/tool_forces", 100, &PsmForceControl::callback_force,this);
    setforce_sub=n.subscribe("/psm_sense/setforce",100, &PsmForceControl::CallbackSetForce,this);
    setpos_sub=n.subscribe("/psm/cmd_vel2",100, &PsmForceControl::CallbackSetPosition,this);
    setpos_sub2=n.subscribe("/psm/cmd_vel",100, &PsmForceControl::CallbackSetPositionIncrement,this);

    q.resize(3); qd.resize(3); eff.resize(3); xe.resize(3); ve.resize(3); fd.resize(3);
    he.resize(3); xf.resize(3); xd.resize(3); vd.resize(3); ad.resize(3); y.resize(3); u.resize(3); x0.resize(3); q0.resize(3);
    x_int.resize(3); v_int.resize(3); a_int.resize(3);

    Ja.resize(3,3); JaM.resize(3,3); Jd.resize(3,3); Jmin.resize(3,3);
    N.resize(3); G.resize(3); C.resize(3,3); Fr.resize(3);
    M.resize(3,3); Mt.resize(3,3); Kp.resize(3,3); Kd.resize(3,3); Cp.resize(3,3); Ci.resize(3,3);

    //JointMsgs
    joint_msg.name.push_back("Joint Publisher");
    for (int j=0;j<6;j++)
    { joint_msg.effort.push_back(0.0);
      msg2.velocity.push_back(0.0);
    }



    rate = 30;
    tf = 0.2 ; // moving 0.001 m in 0.2 s is pretty good for u values.

    q1_traj.ts = 1/rate;
    q2_traj.ts = 1/rate;
    q3_traj.ts = 1/rate;

    q1_traj.tf = tf;
    q2_traj.tf = tf;
    q3_traj.tf = tf;

    q1_traj.check = false;
    q2_traj.check = false;
    q3_traj.check = false;

    q1_traj.qd.resize(6);
    q2_traj.qd.resize(6);
    q3_traj.qd.resize(6);

    // Initialize to zero
    fd<< 0.0, 0.0, 0.0;
    he<< 0.0, 0.0, 0.0;

    xd<< 0.0, 0.0, 0.0;
    x0<<0.0,0.0,0.0;
    q0<<0.0,0.0,0.0;

    q<<0.0, 0.0, 0.0;
    qd<< 0.0, 0.0, 0.0;

      C<< 0, 0, 0, 0, 0, 0 ,0 ,0 ,0;
      G <<0 ,0 ,0;
      Fr << 0 ,0 ,0;
      N << 0, 0, 0;

    M<< 0, 0, 0, 0, 0, 0 ,0 ,0 ,0;
    Mt<< 0, 0, 0, 0, 0, 0 ,0 ,0 ,0;
    Ja << 0, 0, 0, 0, 0, 0 ,0 ,0 ,0;
    JaM << 0, 0, 0, 0, 0, 0 ,0 ,0 ,0;
    Jd << 0, 0, 0, 0, 0, 0 ,0 ,0 ,0;

    Kd<< 0, 0,0, 0, 0, 0, 0 ,0 ,0;
    Kp<< 0, 0,0, 0, 0, 0, 0 ,0 ,0;

  }

  //void showImage(const sensor_msgs::ImageConstPtr& img);
  //void getECMEndEffector(const gazebo_msgs::LinkStatesPtr &msg);

  void SetGainsInit();
  void SetDesiredInit();


  void callback_jacobian(std_msgs::Float64MultiArray msg);
  void callback_joint(sensor_msgs::JointState msg);
  void callback_cartesian(geometry_msgs::PoseStamped msg);
  void callback_force(geometry_msgs::Wrench msg);
  void CallbackSetForce(geometry_msgs::Pose msg);
  void CallbackSetPositionIncrement(geometry_msgs::Twist msg);
  void CallbackSetPosition(geometry_msgs::Twist msg);

  void CalcJaM(Eigen::VectorXd q, Eigen::VectorXd qd);
  void CalcDiffJacobian(Eigen::VectorXd q, Eigen::VectorXd  qd);

  void CalcN(Eigen::VectorXd q, Eigen::VectorXd qd);
  void CalcFr(Eigen::VectorXd qd);
  void CalcC(Eigen::VectorXd q, Eigen::VectorXd qd);
  void CalcG(Eigen::VectorXd q);

  void CalcM(Eigen::VectorXd q);

  void CalcU();
  void publish_joint_torque();
  void output();

  void Interpolate(Eigen::VectorXd x0, Eigen::VectorXd xf);

};
