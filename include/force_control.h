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
#include "Interpolate.h"

using namespace std;

class PsmForceControl{
private:
  //std::vector<ros::Publisher> ecmPub, psm1Pub, psm2Pub, psm3Pub;
    ros::Subscriber jacobian_sub, joint_sub, cartesian_sub, force_sub, setforce_sub, setpos_sub, setpos_sub2;
    ros::Publisher plot_x, plot_y, plot_z, joint_pub, desplot_x, desplot_y, desplot_z;

  //std::vector<ros::Publisher> cartPub;
public:
    double rate;
    int t;
    double t0;
    double tf;
    int index;
    string name;

    double sum[3];
    int filter_n;
    int drop;
    int drop_p;

    bool interp;

    std::deque<double> myq[3];
    std::deque<double> que1, que2, que3;

    traject q_traj[3], q1_traj, q2_traj, q3_traj;

    Eigen::VectorXd q, qd, eff,xe, ve, fd, he, xf, xd, vd,ad, y, u, x0, q0, N, x_int, v_int, a_int, G, Fr, deadband, joint_angle;
    Eigen::MatrixXd Ja, JaM, Jd, Jmin , C;
    Eigen::MatrixXd M, Mt, Kp, Kd, Cp, Ci;

  //Eigen::Matrix4d Ja;

    std_msgs::Float64 mq0, mq1, mq2, dq0, dq1, dq2;
    sensor_msgs::JointState joint_msg, msg2;

    PsmForceControl(ros::NodeHandle n, string nam){

        name = nam;

        desplot_x=n.advertise<std_msgs::Float64>("/d0",10);
        desplot_y=n.advertise<std_msgs::Float64>("/d1",10);
        desplot_z=n.advertise<std_msgs::Float64>("/d2",10);

        plot_x=n.advertise<std_msgs::Float64>("/0",10);
        plot_y=n.advertise<std_msgs::Float64>("/1",10);
        plot_z=n.advertise<std_msgs::Float64>("/2",10);

        joint_pub=n.advertise<sensor_msgs::JointState>("/dvrk/" + name + "/set_effort_joint",1);

        //jacobian_sub=n.subscribe("/dvrk/"+ name + "/jacobian_body", 200, &PsmForceControl::CallbackJacobian,this);
        joint_sub=n.subscribe("/dvrk/"+ name + "/state_joint_current", 1, &PsmForceControl::CallbackJoint,this);
        cartesian_sub=n.subscribe("/dvrk/"+ name + "/position_cartesian_current", 1, &PsmForceControl::CallbackCartesian,this);

        force_sub=n.subscribe("/psm_sense/" + name + "/tool_forces", 10, &PsmForceControl::CallbackForce,this);
        setforce_sub=n.subscribe("/psm_sense/setforce",10, &PsmForceControl::CallbackSetForce,this);
        setpos_sub=n.subscribe("/psm/cmd_vel2",10, &PsmForceControl::CallbackSetPosition,this);
        setpos_sub2=n.subscribe("/psm/cmd_vel",10, &PsmForceControl::CallbackSetPositionIncrement,this);

        q.resize(3); qd.resize(3); eff.resize(3); xe.resize(3); ve.resize(3); fd.resize(3);
        he.resize(3); xf.resize(3); xd.resize(3); vd.resize(3); ad.resize(3); y.resize(3); u.resize(3); x0.resize(3); q0.resize(3);
        x_int.resize(3); v_int.resize(3); a_int.resize(3), deadband.resize(3), joint_angle.resize(3);

        Ja.resize(3,3); JaM.resize(3,3); Jd.resize(3,3); Jmin.resize(3,3);
        N.resize(3); G.resize(3); C.resize(3,3); Fr.resize(3);
        M.resize(3,3); Mt.resize(3,3); Kp.resize(3,3); Kd.resize(3,3); Cp.resize(3,3); Ci.resize(3,3);

        //JointMsgs
        joint_msg.name.push_back("Joint Publisher");
        for (int j=0;j<6;j++)
        { joint_msg.effort.push_back(0.0);
          msg2.velocity.push_back(0.0);
        }


        myq[0]= que1;
        myq[1]= que2;
        myq[2]= que3;

        rate = 2000;
        tf = 1; // moving 0.001 m in 0.2 s is pretty good for u values.
        filter_n = 20;
        index = 0;

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

        q_traj[0] = q1_traj;
        q_traj[1] = q2_traj;
        q_traj[2] = q3_traj;

        // Initialize to zero
        fd << 0.0, 0.0, 0.0;
        he << 0.0, 0.0, 0.0;

        xd << 0.0, 0.0, 0.0;
        xe << 0.0, 0.0, 0.0;

        x0 << 0.0, 0.0, 0.0;
        q0 << 0.0, 0.0, 0.0;

        q  << 0.0, 0.0, 0.0;
        qd << 0.0, 0.0, 0.0;

        C  << 0, 0, 0, 0, 0, 0 ,0 ,0 ,0;
        G  << 0, 0, 0;
        Fr << 0, 0, 0;
        N  << 0, 0, 0;

        M  << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        Mt << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        Ja << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        JaM<< 0, 0, 0, 0, 0, 0, 0, 0, 0;
        Jd << 0, 0, 0, 0, 0, 0, 0, 0, 0;

        Kd << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        Kp << 0, 0, 0, 0, 0, 0, 0, 0, 0;

        deadband << 0.005, 0.01, 0.005;

        interp = false;
  }

  //void showImage(const sensor_msgs::ImageConstPtr& img);
  //void getECMEndEffector(const gazebo_msgs::LinkStatesPtr &msg);

  void SetGainsInit();
  void SetDesiredInit();

  void CallbackJacobian(std_msgs::Float64MultiArray msg);
  void CallbackJoint(sensor_msgs::JointState msg);
  void CallbackCartesian(geometry_msgs::PoseStamped msg);
  void CallbackForce(geometry_msgs::Wrench msg);
  void CallbackSetForce(geometry_msgs::Pose msg);
  void CallbackSetPositionIncrement(geometry_msgs::Twist msg);
  void CallbackSetPosition(geometry_msgs::Twist msg);

  void CalcJaM(Eigen::VectorXd q, Eigen::VectorXd qd);
  void CalcDiffJacobian(Eigen::VectorXd q, Eigen::VectorXd  qd);

  void CalcN(Eigen::VectorXd q, Eigen::VectorXd qd);
  void CalcFr(Eigen::VectorXd q, Eigen::VectorXd qd);
  void CalcC(Eigen::VectorXd q, Eigen::VectorXd qd);
  void CalcG(Eigen::VectorXd q);
  void CalcM(Eigen::VectorXd q);

  void CalcU();
  void InverseKinematic();
  void output();

};
