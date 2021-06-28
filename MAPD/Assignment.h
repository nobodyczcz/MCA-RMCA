//
// Created by Zhe Chen on 6/8/20.
//

#include "Task.h"
#include "Agent.h"

#ifndef MAPD_ASSIGNMENT_H
#define MAPD_ASSIGNMENT_H
struct ActionEntry{
    ActionEntry(Task* task1,ACTION_TYPE action,int loc,int time,int load = 0)
            :task(task1),action_type(action),location(loc),release_time(time),current_load(load){
        if (task1!=NULL)
            task_id = task1->task_id;
        else
            task_id = -1;

        if (action == ACTION_TYPE::START) {
            ideal_action_time = 0;
            release_time = 0;
            current_load = 0;
            current_total_delay = 0;
        }

        if (action == ACTION_TYPE::DOCK) {
            ideal_action_time = 1;
            release_time = 1;
            current_load = 0;
            current_total_delay = 0;
            task = NULL;
        }
    }
    int task_id;
    Task* task;
    ACTION_TYPE action_type;
    int location=0;
    int release_time=0;
    int ideal_action_time=0;
    int current_total_delay=0;
    int real_current_total_delay=0;
    int current_load=0;
    int real_action_time=0;
};

struct Assignment{
    vector<ActionEntry> actions;
    std::unordered_set<Task*> assigned_tasks;
    Agent* agent;
    int ideal_cost = 0;
    int current_total_delay=0;
    vector<PathEntry> path;
    int path_length=0;
    int makespan = 0;
    int start_action = 0;


    //for new assignment
    float cost_increase;
    Task* new_add_task;
    int insert_at;

    int current_target_index;
    bool done = false;



    void print(){
        for (auto& a: actions){
            cout<<"<"<<a.ideal_action_time<<",("<<a.location/map_cols<<","<<a.location%map_cols<<"),delay"
                <<a.current_total_delay<<",act"<<a.action_type<<",r"<<a.release_time<<">,";
        }
        cout<<endl;
    }

    struct compare_assignment
    {
        // returns true if t1 > t2 (note -- this gives us *min*-heap).
        bool operator()(const Assignment* t1, const Assignment* t2) const
        {
            if (t1->cost_increase == t2->cost_increase){
                if (t1->path.size() == t2->path.size()){
                    if( t1->ideal_cost == t2->ideal_cost){
                        return t1->new_add_task->initial_time >= t2->new_add_task->initial_time;
                    }
                    return t1->ideal_cost > t2->ideal_cost;
                }
                else
                    return t1->path.size() > t2->path.size();
            }
            else
                return t1->cost_increase > t2->cost_increase;

        }
    };
    bool static compare_assignment_delay (Assignment& i,Assignment& j) { return (i.current_total_delay>j.current_total_delay); }
    bool static compare_assignment_span (Assignment& i,Assignment& j) { return (i.makespan>j.makespan); }


    typedef boost::heap::fibonacci_heap< Assignment*, boost::heap::compare<Assignment::compare_assignment>>::handle_type AssignmentHeap_handle;
    AssignmentHeap_handle heap_handle;

};






typedef boost::heap::fibonacci_heap< Assignment*, boost::heap::compare<Assignment::compare_assignment>> AssignmentHeap;
typedef boost::heap::fibonacci_heap< Assignment*, boost::heap::compare<Assignment::compare_assignment>>::handle_type AssignmentHeap_handle;

struct compare_assignment_heap {
    // returns true if t1 > t2 (note -- this gives us *min*-heap).
    bool operator()(const AssignmentHeap *t1, const AssignmentHeap *t2) const {
        if (t1->top()->cost_increase == t2->top()->cost_increase) {
            if (t1->top()->path.size() == t2->top()->path.size()){
                if( t1->top()->ideal_cost == t2->top()->ideal_cost){
                    return t1->top()->new_add_task->initial_time >= t2->top()->new_add_task->initial_time;
                }
                return t1->top()->ideal_cost > t2->top()->ideal_cost;
            }
            else
                return t1->top()->path.size() > t2->top()->path.size();
        } else
            return t1->top()->cost_increase > t2->top()->cost_increase;

    }
};

struct compare_assignment_heap_regret_old {
// returns true if t1 > t2 (note -- this gives us *min*-heap).
    bool operator()(const AssignmentHeap *t1, const AssignmentHeap *t2) const {
        int t1_top_diff;
        auto it1 = t1->ordered_begin();
        Assignment *t1_first = *(it1);
        it1++;
        if (it1 != t1->ordered_end()) {
            Assignment *t1_second = *(it1);
            t1_top_diff = t1_second->cost_increase - t1_first->cost_increase;
        } else {
            t1_top_diff = 0;
        }

        int t2_top_diff;
        auto it2 = t2->ordered_begin();
        Assignment *t2_first = *(it2);
        it2++;
        if (it2 != t2->ordered_end()) {
            Assignment *t2_second = *(it2);
            t2_top_diff = t2_second->cost_increase - t2_first->cost_increase;
        } else {
            t2_top_diff = 0;
        }

        if (t1_top_diff == t2_top_diff) {
            if (t1->top()->cost_increase == t2->top()->cost_increase) {

                if (t1->top()->path.size() == t2->top()->path.size()) {
                    if (t1->top()->ideal_cost == t2->top()->ideal_cost) {
                        return t1->top()->new_add_task->initial_time >= t2->top()->new_add_task->initial_time;
                    }
                    return t1->top()->ideal_cost > t2->top()->ideal_cost;
                } else
                    return t1->top()->path.size() > t2->top()->path.size();

            } else
                return t1->top()->cost_increase > t2->top()->cost_increase;
        } else {
            return t1_top_diff <= t2_top_diff;
        }
    }
};


struct compare_assignment_heap_regret {
// returns true if t1 > t2 (note -- this gives us *min*-heap).
    bool operator()(const AssignmentHeap *t1, const AssignmentHeap *t2) const {
        int t1_top_diff;
        auto it1 = t1->ordered_begin();
        Assignment* t1_first = *(it1);
        it1++;
        if (it1!=t1->ordered_end()) {
            Assignment *t1_second = *(it1);
            t1_top_diff = ((t1_second->cost_increase+1)/((float)t1_first->cost_increase+1)) * 100;
        }
        else{
            t1_top_diff = 0;
        }

        int t2_top_diff;
        auto it2 = t2->ordered_begin();
        Assignment* t2_first = *(it2);
        it2++;
        if (it2!=t2->ordered_end()) {
            Assignment *t2_second = *(it2);
            t2_top_diff = ((t2_second->cost_increase+1)/((float)t2_first->cost_increase+1)) * 100;
        }
        else{
            t2_top_diff = 0;
        }

        if (t1_top_diff == t2_top_diff){
            if (t1->top()->cost_increase == t2->top()->cost_increase) {

                if (t1->top()->path.size() == t2->top()->path.size()){
                    if( t1->top()->ideal_cost == t2->top()->ideal_cost){
                        return t1->top()->new_add_task->initial_time >= t2->top()->new_add_task->initial_time;
                    }
                    return t1->top()->ideal_cost > t2->top()->ideal_cost;
                }
                else
                    return t1->top()->path.size() > t2->top()->path.size();

            } else
                return t1->top()->cost_increase > t2->top()->cost_increase;
        }
        else{
            return t1_top_diff <= t2_top_diff;
        }

//            if (t1->top()->cost_increase/10 == t2->top()->cost_increase/10) {
//                if (t1_top_diff == t2_top_diff){
//                    if (t1->top()->cost_increase == t2->top()->cost_increase) {
//                        return t1->top()->path.size() > t2->top()->path.size();
//                    } else
//                        return t1->top()->cost_increase > t2->top()->cost_increase;
//                }
//                else{
//                    return t1_top_diff <= t2_top_diff;
//                }
//            } else
//                return t1->top()->cost_increase/10 > t2->top()->cost_increase/10;


//            if (t1->top()->cost_increase == t2->top()->cost_increase) {
//                if (t1_top_diff == t2_top_diff){
//                    return t1->top()->path.size() > t2->top()->path.size();
//                }
//                else{
//                    return t1_top_diff <= t2_top_diff;
//                }
//            } else
//                return t1->top()->cost_increase > t2->top()->cost_increase;



    }
};

struct compare_assignment_heap_regret_minus {
// returns true if t1 > t2 (note -- this gives us *min*-heap).
    bool operator()(const AssignmentHeap *t1, const AssignmentHeap *t2) const {
        int t1_top_diff;
        auto it1 = t1->ordered_begin();
        Assignment* t1_first = *(it1);
        it1++;
        if (it1!=t1->ordered_end()) {
            Assignment *t1_second = *(it1);
            t1_top_diff = t1_second->cost_increase - t1_first->cost_increase ;
        }
        else{
            t1_top_diff = 0;
        }

        int t2_top_diff;
        auto it2 = t2->ordered_begin();
        Assignment* t2_first = *(it2);
        it2++;
        if (it2!=t2->ordered_end()) {
            Assignment *t2_second = *(it2);
            t2_top_diff = t2_second->cost_increase - t2_first->cost_increase;
        }
        else{
            t2_top_diff = 0;
        }

        if (t1_top_diff == t2_top_diff){
            if (t1->top()->cost_increase == t2->top()->cost_increase) {

                if (t1->top()->path.size() == t2->top()->path.size()){
                    if( t1->top()->ideal_cost == t2->top()->ideal_cost){
                        return t1->top()->new_add_task->initial_time >= t2->top()->new_add_task->initial_time;
                    }
                    return t1->top()->ideal_cost > t2->top()->ideal_cost;
                }
                else
                    return t1->top()->path.size() > t2->top()->path.size();

            } else
                return t1->top()->cost_increase > t2->top()->cost_increase;
        }
        else{
            return t1_top_diff <= t2_top_diff;
        }

    }
};

struct compare_assignment_heap_regret_partial {
// returns true if t1 > t2 (note -- this gives us *min*-heap).
    bool operator()(const AssignmentHeap *t1, const AssignmentHeap *t2) const {
        int t1_top_diff;
        auto it1 = t1->ordered_begin();
        Assignment* t1_first = *(it1);
        it1++;
        if (it1!=t1->ordered_end()) {
            Assignment *t1_second = *(it1);
            t1_top_diff = (t1_second->cost_increase - t1_first->cost_increase);
        }
        else{
            t1_top_diff = 0;
        }

        int t2_top_diff;
        auto it2 = t2->ordered_begin();
        Assignment* t2_first = *(it2);
        it2++;
        if (it2!=t2->ordered_end()) {
            Assignment *t2_second = *(it2);
            t2_top_diff = (t2_second->cost_increase - t2_first->cost_increase);
        }
        else{
            t2_top_diff = INT_MAX;
        }


            if (t1->top()->cost_increase/10 == t2->top()->cost_increase/10) {
                if (t1_top_diff == t2_top_diff){
                    if (t1->top()->cost_increase == t2->top()->cost_increase) {
                        if (t1->top()->path.size() == t2->top()->path.size()){
                            if( t1->top()->ideal_cost == t2->top()->ideal_cost){
                                return t1->top()->new_add_task->initial_time >= t2->top()->new_add_task->initial_time;
                            }
                            return t1->top()->ideal_cost > t2->top()->ideal_cost;
                        }
                        else
                            return t1->top()->path.size() > t2->top()->path.size();
                    } else
                        return t1->top()->cost_increase > t2->top()->cost_increase;
                }
                else{
                    return t1_top_diff <= t2_top_diff;
                }
            } else
                return t1->top()->cost_increase/10 > t2->top()->cost_increase/10;


    }
};


#endif //MAPD_ASSIGNMENT_H
