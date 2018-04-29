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

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "std_msgs/String.h"

class PsmForceControl{
private:
  //std::vector<ros::Publisher> ecmPub, psm1Pub, psm2Pub, psm3Pub;
  ros::Subscriber jacobian_sub, joint_sub, cartesian_sub, force_sub, setforce_sub, setpos_sub, setpos_sub2;
  ros::Publisher plot_x, plot_y, plot_z, joint_pub;

  //std::vector<ros::Publisher> cartPub;
public:


  Eigen::VectorXd q, qd, eff,xe, ve, fd, he, xf, xd, vd,ad, y, u, x0, q0, N;
  Eigen::MatrixXd Ja, Jd, Jmin ;
  Eigen::MatrixXd M, Kp, Kd, Cp, Ci;

  //Eigen::Matrix4d Ja;

  std_msgs::Float64 msg2;
  sensor_msgs::JointState joint_msg;

  PsmForceControl(ros::NodeHandle n){
    plot_x=n.advertise<std_msgs::Float64>("/Jd",10);
    joint_pub=n.advertise<sensor_msgs::JointState>("/dvrk/PSM3/SetEffortJoint",10);

    jacobian_sub=n.subscribe("/dvrk/PSM1/jacobian_body", 100, &PsmForceControl::callback_jacobian,this);
    joint_sub=n.subscribe("/dvrk/PSM1/state_joint_current", 100, &PsmForceControl::callback_joint,this);
    cartesian_sub=n.subscribe("/dvrk/PSM1/position_cartesian_current", 100, &PsmForceControl::callback_cartesian,this);

    force_sub=n.subscribe("/psm_sense/PSM2/tool_forces", 100, &PsmForceControl::callback_force,this);
    setforce_sub=n.subscribe("/psm_sense/setforce",100, &PsmForceControl::CallbackSetForce,this);
    setpos_sub=n.subscribe("/psm_sense/setposition",100, &PsmForceControl::CallbackSetPosition,this);
    setpos_sub2=n.subscribe("/psm_sense/setposition_increment",100, &PsmForceControl::CallbackSetPositionIncrement,this);

    q.resize(3); qd.resize(3); eff.resize(3); xe.resize(3); ve.resize(3); fd.resize(3);
    he.resize(3); xf.resize(3); xd.resize(3); vd.resize(3); ad.resize(3); y.resize(3); u.resize(3); x0.resize(3); q0.resize(3);

    Ja.resize(3,3); Jd.resize(3,3); Jmin.resize(3,3), N.resize(3);
    M.resize(3,3); Kp.resize(3,3); Kd.resize(3,3); Cp.resize(3,3); Ci.resize(3,3);

    //JointMsgs
    joint_msg.name.push_back("Joint Publisher");
    for (int j=0;j<7;j++)
    { joint_msg.effort.push_back(0.0);}

    // Initialize to zero
    fd<< 0.0, 0.0, 0.0;
    he<< 0.0, 0.0, 0.0;

    xd<< 0.0, 0.0, 0.0;
    x0<<0.0,0.0,0.0;
    q0<<0.0,0.0,0.0;

    q<<0.0, 0.0, 0.0;
    qd<< 0.0, 0.0, 0.0;

    M<< 0, 0, 0, 0, 0, 0 ,0 ,0 ,0;

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
  void CallbackSetPositionIncrement(geometry_msgs::Pose msg);
  void CallbackSetPosition(geometry_msgs::Pose msg);
  void CalcN(Eigen::VectorXd q, Eigen::VectorXd qd);
  void CalcDiffJacobian(Eigen::VectorXd q, Eigen::VectorXd  qd);
  void CalcM(Eigen::VectorXd q);
  void Interpolate(Eigen::VectorXd x0, Eigen::VectorXd xf);
  void CalcU();
  void publish_joint_torque();
  void output();

};
