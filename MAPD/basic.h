//
// Created by Zhe Chen on 10/6/20.
//


#include <map>

#ifndef MAPD_COMMON_H
#define MAPD_COMMON_H
enum ACTION_TYPE {START,PICK_UP,DROP_OFF,DOCK};
enum OBJECTIVE {TOTAL_TRAVEL_COST,TOTAL_TRAVEL_DELAY,MAKESPAN};

enum DESTORY {FROM_MAX,MULTI_MAX, RANDOM};
enum TaskState {NONE,TRANSPORTING,DONE};

struct TA_Options{
    OBJECTIVE objective;
    bool no_task_loop = false;
    bool only_top = false;
    bool real_cost_insert = false;
    bool rc_first = true;
    int time_limit = 0;
    int group_size = 1;
    int max_iteration = 10000;
    int improve_time_limit = 0;
    DESTORY destory_method;

};

#endif //MAPD_COMMON_H
