//
// Created by radian on 7/22/18.
//

#ifndef PROJECT_PATH_PLANNER_H
#define PROJECT_PATH_PLANNER_H


#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <psm_coop/DataType.h>
#include <psm_coop/psm_sub.h>
#include <psm_coop/csv_writer.h>

class PathPlanner {
private:

    std::vector<initializer> psm_data;

    std::shared_ptr<ros::NodeHandle> nhandle;
    int Psm_num;

    std::vector<std::shared_ptr<PsmSub>> Psm;

    Path path;

    double t;
    double rate;

public:
    PathPlanner(std::vector<initializer> &psm);
    void CalculatePath();
    void RotatePath();
    void DesiredPath();
    int Loop();
    void SaveHistory();
    void Write();

};

#endif //PROJECT_PATH_PLANNER_H