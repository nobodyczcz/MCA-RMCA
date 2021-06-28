//
// Created by Zhe Chen on 10/6/20.
//

#include <vector>
#include "map_loader_with_cost.h"
#include "basic.h"
#include<boost/tokenizer.hpp>
#include <utility>
#include <iostream>
#include <fstream>



#ifndef LIFELONG_MAPF_TASK_H
#define LIFELONG_MAPF_TASK_H
using namespace::std;
using namespace::boost;
extern int map_cols;


class Task {
public:
    Task(int id,int init_t, int init_loc, int goal_loc);

    int initial_time;
    int initial_location;
    int goal_location;
    int task_id;
    int ideal_end_time;
    TaskState state= TaskState::NONE;

    struct compare_task
    {
        // returns true if t1 > t2 (note -- this gives us *min*-heap).
        bool operator()(const Task* t1, const Task* t2) const
        {
            if (t1->initial_time>= t2->initial_time)
                return true;
            else
                return false;
        }
    };

    typedef boost::heap::fibonacci_heap< Task*, boost::heap::compare<Task::compare_task> >::handle_type TaskHeap_handle;
    TaskHeap_handle heap_handle;
};

typedef boost::heap::fibonacci_heap< Task*, boost::heap::compare<Task::compare_task> > TaskHeap;




class TaskLoader{
public:
    TaskLoader(const std::string fname, MapLoaderCost &ml);
    TaskLoader(){};
    void loadKiva(const std::string fname, MapLoaderCost &ml);
    int num_of_tasks;
    int last_release_time = 0;
    TaskHeap all_tasks;
    vector<Task*> all_tasks_vec;
    void printTasks(){
        for(auto t : all_tasks){
            cout << "Task:" <<t->task_id << ", release_time:" <<t->initial_time
            <<", Initial:(" <<t->initial_location/map_cols<<","<<t->initial_location%map_cols<<"),"
            <<" Goal :(" <<t->goal_location/map_cols<<","<<t->goal_location%map_cols<<"),"<<endl;
        }
    }

    ~TaskLoader(){
        for (auto t : all_tasks){
            delete t;
        }
    }
};
#endif //LIFELONG_MAPF_TASK_H

