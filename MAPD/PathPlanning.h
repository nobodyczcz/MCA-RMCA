//
// Created by Zhe Chen on 10/6/20.
//
#include "basic.h"
#include "Assignment.h"
#include "map_loader.h"
#include "CBSHOnline.h"

#ifndef MAPD_PATHPLANNING_H
#define MAPD_PATHPLANNING_H


struct Task_info{
    Task* task;
    int start_time;
    int end_time;
    int delay;

};

class PathPlanner{
public:
    PathPlanner(vector<Assignment>& assignments,float cut_off, constraint_strategy s,int screen,options option,MapLoaderCost* mapLoader);

    bool startPlan();
    void printPath();
    bool checkResult();


    vector<Assignment>& assignments;
    vector<vector<PathEntry>> plans;
    double runtime;
    boost::unordered::unordered_map<Task*, Task_info> task_info_table;

    int num_replan=0;
protected:
    int window_size=0;
    bool pbs=true;
    constraint_strategy s;
    options cbs_option;
    int screen;
    float cut_off;
    MapLoaderCost* mapLoader;


};
#endif //MAPD_PATHPLANNING_H
