//
// Created by Zhe Chen on 10/6/20.
//

#ifndef LIFELONG_MAPF_TaskAssignmentRegretTask_H
#define LIFELONG_MAPF_TaskAssignmentRegretTask_H

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


class TaskAssignmentRegretTask: public TaskAssignment{
public:


    typedef boost::heap::fibonacci_heap<AssignmentHeap*,boost::heap::compare<compare_assignment_heap_regret>> Agent_heap_regret;
    typedef boost::heap::fibonacci_heap<AssignmentHeap*,boost::heap::compare<compare_assignment_heap_regret>>::handle_type  AgentHeap_handle_regret;

    TaskAssignmentRegretTask(AgentLoader* agents, TaskLoader* tasks, MapLoaderCost* map, TA_Options ta_opt, options pl_option, bool real_cost, float pl_time_limit)
        :TaskAssignment(agents, tasks, map, ta_opt, pl_option, real_cost, pl_time_limit){};

    void initialize_heaps(){
        exit(255);
    };

    Assignment* get_best_assignment(){return allAssignmentHeap.top()->top();};
    void delete_top(int task_id){
        Assignment* assign = allAssignmentHeap.top()->top();
        allAssignmentHeap.top()->pop();
        delete assign;
    }
    void logAssignmentHandle(Task* task,Agent* a,AssignmentHeap* new_assignment_heap){
        allAssignmentHandles[a->agent_id] = allAssignmentHeap.push(new_assignment_heap);
    };
    bool updateRealPath(Assignment* min_cost_assign,Agent* a,int next_action = 1){
        min_cost_assign->current_total_delay = prioritized_planning(min_cost_assign->path, min_cost_assign->actions, min_cost_assign->agent,next_action);
        min_cost_assign->cost_increase = min_cost_assign->current_total_delay - assignments[a->agent_id].current_total_delay;
        if(min_cost_assign->cost_increase<0)
            min_cost_assign->cost_increase = 1.0/(float)abs(min_cost_assign->cost_increase);

        if (ta_option.objective == OBJECTIVE::MAKESPAN) {
            min_cost_assign->makespan = min_cost_assign->actions[min_cost_assign->actions.size() - 2].real_action_time;
            min_cost_assign->cost_increase =
                    ( min_cost_assign->makespan -
                      current_makespan) > 0 ? (min_cost_assign->makespan - current_makespan) : 0;
        }
        return true;
    }

    void buildAssignmentHeap() ;

    void updateAllAssignmentHeap(Agent* updatedAgent,Task* assignedTask);
    void printTaskHeap(){
        for(auto it = allAssignmentHeap.ordered_begin();it!= allAssignmentHeap.ordered_end();++it){
            auto agentHeap = (*it);
            cout<<" Agent : "<<agentHeap->top()->agent->agent_id;
            cout<<" MC : "<<agentHeap->top()->cost_increase;

            int t1_top_diff;
            auto it1 = agentHeap->ordered_begin();
            Assignment* t1_first = *(it1);
            it1++;
            if (it1!=agentHeap->ordered_end()) {
                Assignment *t1_second = *(it1);
                t1_top_diff = t1_second->cost_increase/(float)t1_first->cost_increase * 100;
            }
            else{
                t1_top_diff = 0;
            }

            cout<<" RC: "<<t1_top_diff;


            for (auto agit = agentHeap->ordered_begin();agit!=agentHeap->ordered_end();++agit){
                auto assign = (*agit);
                cout<<", Task: "<<assign->new_add_task->task_id<<" MC: "<<assign->cost_increase;
            }
            cout<<endl;
        }
    }


    Agent_heap_regret allAssignmentHeap;
    std::unordered_map<int,AgentHeap_handle_regret> allAssignmentHandles;





};

#endif //LIFELONG_MAPF_TaskAssignmentRegretTask_H
