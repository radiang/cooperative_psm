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

    t = 0;

    rate = psm_data[0].rate;

    this->DesiredPath();
    this->CalculatePath();

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
    path.checkpoint = 0;
    path.delta.resize(path.num_points);
}

void PathPlanner::CalculatePath(){
    for(int i;i<(path.num_points-1);i++) {
        path.delta[i] = (path.set_points[i+1]-path.set_points[i])/rate;
    }
}

void PathPlanner::RotatePath() {

    int iter= path.checkpoint;

    for(int i;i<Psm_num;i++) {


        if (psm_data[i].type == "Master"){
            Psm[i]->xd = path.delta[iter] + Psm[i]->xd;
        }
        else if (psm_data[i].type =="Slave")
        {
            Psm[i]->xd = psm_data[i].Rot.inverse()*path.delta[iter] + Psm[i]->xd;
        }

    }
}

int PathPlanner::Loop() {

    this->RotatePath();

    for(int i;i<Psm_num;i++) {
        Psm[i]->PublishXd();
    }

    //
    this->SaveHistory();


    t = t + 1/rate;

    if(t == path.set_times[path.checkpoint+1]){
        path.checkpoint =+1;

        if(path.checkpoint==path.num_points){
            return 1;
        }
        else {
            return 0;
        }
    }
}

void PathPlanner::SaveHistory() {
    for(int i;i<Psm_num;i++){
        int x = 0;
    }
}

/*void PathPlanner::Write(){

}*/

void PathPlanner::Write(string &file_name, vector<Eigen::Vector3d> *v_j)
{
    CSVWriter writer(file_name);
    vector<double> v3 = {0, 0, 0};

    for(auto it = v_j->begin(); it != v_j->end(); ++it)
    {
        v3[0] = it->data()[0];
        v3[1] = it->data()[1];
        v3[2] = it->data()[2];
        writer.addDatainRow(v3.begin(), v3.end());
    }
}
