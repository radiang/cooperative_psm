//
// Created by radian on 7/22/18.
//

#include <psm_coop/path_planner.h>

PathPlanner::PathPlanner(std::vector<initializer> &psm){

    psm_data = psm;

    Psm_num = psm_data.size();
    nhandle = std::make_shared<ros::NodeHandle>();

    Psm.resize(Psm_num);

    for(int i;i<Psm_num;i++) {
        Psm[i]=std::make_shared<PsmSub>(nhandle, psm_data[i].name);

        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ros::spinOnce();

        Psm[i]->GetInit();
    }

    Eigen::Vector3d x;
    x<<0,0,0;

    path.set_points.push_back(x);
    path.set_times.push_back(0);

}

void PathPlanner::DesiredPath(){

    Eigen::Vector3d x(0,0.05,0);
    double ts = 5;

    path.set_points.push_back(x);
    path.set_times.push_back(ts);

    x(0) = 0.05;
    x(1) = 0.05;
    x(2) = 0;
    ts = 10;

    path.set_points.push_back(x);
    path.set_times.push_back(ts);

    x(0) = 0.05;
    x(1) = 0.05;
    x(2) = 0.05;
    ts = 15;

    path.set_points.push_back(x);
    path.set_times.push_back(ts);

    path.num_points = path.set_points.size();
}

void PathPlanner::CalculatePath() {

    

    for(int i;i<Psm_num;i++) {
        if (psm_data[i].type == "Master"){
            Psm[i]->xd = Psm[i]->xd;
        }
        else if (psm_data[i].type =="Slave")
        {
            Psm[i]->xd = psm_data[i].Rot.inverse() * Psm[i]->xd;
        }
    }
}

void PathPlanner::Loop() {
    for(int i;i<Psm_num;i++) {
        Psm[i]->PublishXd();
    }
}



