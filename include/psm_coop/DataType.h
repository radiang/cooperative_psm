//
// Created by radian on 6/29/18.
//

#ifndef PROJECT_DATATYPE_H
#define PROJECT_DATATYPE_H

#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;

struct traject {
    Eigen::VectorXd x;
    Eigen::VectorXd v;
    Eigen::VectorXd a;
    double ts;
    double tf;
    bool check;
    bool check2;
    Eigen::VectorXd qd;
};

struct initializer {
    string name;
    string type;
    string ctrl_type;
    bool track;
    Eigen::MatrixXd Rot;
    Eigen::VectorXd Pos;
    ros::NodeHandle n;
    double rate;
};

struct Path {


    std::vector<Eigen::Vector3d> set_points;
    std::vector<double> set_times;
    int num_points;

};

#endif //PROJECT_DATATYPE_H
