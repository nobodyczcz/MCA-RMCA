//
// Created by Zhe Chen on 10/6/20.
//

#ifndef LIFELONG_MAPF_TaskAssignmentTaskHeap_H
#define LIFELONG_MAPF_TaskAssignmentTaskHeap_H

#include "Agent.h"
#include "Task.h"
#include "PathPlanning.h"
#include "Assignment.h"
#include "ConstraintTable.h"
#include <unordered_set>
#include <unordered_map>
#include "common.h"
#include "SinglePlanning.h"
#include "TaskAssignment.h"




class TaskAssignmentTaskHeap: public TaskAssignment{
public:


    typedef boost::heap::fibonacci_heap<AssignmentHeap*,boost::heap::compare<compare_assignment_heap>> Task_heap;
    typedef boost::heap::fibonacci_heap<AssignmentHeap*,boost::heap::compare<compare_assignment_heap>>::handle_type  TaskHeap_handle;

    TaskAssignmentTaskHeap(AgentLoader* agents, TaskLoader* tasks, MapLoaderCost* map, TA_Options ta_opt, options pl_option, bool real_cost, float pl_time_limit)
            :TaskAssignment(agents, tasks, map, ta_opt, pl_option, real_cost, pl_time_limit){};


    void initialize_heaps(){
        handleTable.clear();
        for(auto& t: unassigned_tasks){
            handleTable[t->task_id].resize(agents->num_of_agents);
        }

        allAssignmentHandles.clear();
    };

    Assignment* get_best_assignment(){return allAssignmentHeap.top()->top();};
    void delete_top(int task_id){
        for (auto assign : *(allAssignmentHeap.top())){
            delete assign;
        }
        delete allAssignmentHeap.top();
        allAssignmentHeap.pop();
        handleTable[task_id].clear();

    }
    void logAssignmentHandle(Task* task,Agent* a,AssignmentHeap* new_assignment_heap){
        allAssignmentHandles[task->task_id] = allAssignmentHeap.push(new_assignment_heap);
    };
    bool updateRealPath(Assignment* min_cost_assign,Agent* a,int next_action = 1){
        return true;
    }

    void updateAllAssignmentHeap(Agent* updatedAgent,Task* assignedTask);
    void printTaskHeap(){
        for(auto it = allAssignmentHeap.ordered_begin();it!= allAssignmentHeap.ordered_end();++it){
            auto agentHeap = (*it);
            cout<<" Task : "<<agentHeap->top()->new_add_task->task_id;
            cout<<" MC : "<<agentHeap->top()->cost_increase;


            for (auto agit = agentHeap->ordered_begin();agit!=agentHeap->ordered_end();++agit){
                auto assign = (*agit);
                cout<<", Agent: "<<assign->agent->agent_id<<" MC: "<<assign->cost_increase;
            }
            cout<<endl;
        }
    }



    Task_heap allAssignmentHeap;
    std::unordered_map<int,TaskHeap_handle> allAssignmentHandles;




};

#endif //LIFELONG_MAPF_TaskAssignmentTaskHeap_H
