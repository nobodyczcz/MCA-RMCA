//
// Created by Zhe Chen on 10/6/20.
//

#include "TaskAssignmentPPTask.h"

void TaskAssignmentPPTask::buildAssignmentHeap(){

    for (Agent* a : agents->agents){
        if(screen>=3)
            cout<<"Build heap for Agent: "<<a->agent_id<<endl;
        AssignmentHeap* new_assignment_heap  = new AssignmentHeap();
        for(Task* task : unassigned_tasks ) {
            if(screen>=3)
                cout<<"Build heap for Agent: "<<a->agent_id<<endl;
            Assignment* min_cost_assign =  insertTask(a, task,ta_option.real_cost_insert);
            min_cost_assign->heap_handle = new_assignment_heap->push(min_cost_assign);
            handleTable[a->agent_id][task->task_id] = min_cost_assign->heap_handle;
        }
        logAssignmentHandle(NULL,a, new_assignment_heap);
    }
}

void TaskAssignmentPPTask::updateAllAssignmentHeap(Agent* updatedAgent,Task* assignedTask){
    bool taskheap_have_zero_increase = false;
    if(screen>=3)
        cout<<"Update Task Heap"<<endl;

    for (int i = 0; i < allAssignmentHandles.size(); i++){
        if (handleTable[i].empty()) {
            continue;
        }
        AssignmentHeap* assignHeap = *allAssignmentHandles[i];

        if(ta_option.only_top){

            if (handleTable[i][assignedTask->task_id].node_ != NULL){
                Assignment* assign = *handleTable[i][assignedTask->task_id];
                if (i!=updatedAgent->agent_id){
                    assignHeap->erase(handleTable[i][assignedTask->task_id]);
                    delete assign;
                }
                handleTable[i][assignedTask->task_id].node_ =NULL;
            }

            if (ta_option.objective == OBJECTIVE::MAKESPAN){
                for (int j =0;j<assignHeap->size();j++){
                    if (handleTable[i][j].node_ == NULL)
                        continue;
                    Assignment* assign = *handleTable[i][j];
                    assign->cost_increase = assign->makespan - current_makespan >=0 ? assign->makespan - current_makespan : 0;
                    assignHeap->update(handleTable[i][j]);
                }
            }

            if (updatedAgent->agent_id == i){
                for (int j = 0; j < handleTable[i].size(); j++) {
                    if (handleTable[i][j].node_ == NULL)
                        continue;
                    Assignment* assign = *handleTable[i][j];
                    clock_t t = clock();
                    num_task_assign_updates+=1;
                    Assignment* new_assign =  insertTask(updatedAgent, assign->new_add_task);
                    if (new_assign->current_total_delay <0) {
                        assignHeap->erase(handleTable[i][j]);
                        if (assign !=NULL)
                            delete assign;
                        delete new_assign;
                        handleTable[i][j].node_ = NULL;
                        continue;
                    }
                    else {
                        new_assign->heap_handle = assign->heap_handle;
                        assert(new_assign->actions.size() > assign->actions.size());
                        assignHeap->update(assign->heap_handle, new_assign);
                        if (assign !=NULL)
                            delete assign;
                    }
                    runtime_update_changed_agent += clock() - t;
                }
                continue;
            }




            bool stop = false;
            while (!stop){
                Assignment* assign = assignHeap->top();
                int j = assign->new_add_task->task_id;
                int conflict_time =  haveConflict(assign->agent->agent_id,assign->path);
                if (conflict_time<0) {
                    stop = true;
                    continue;
                }
                clock_t t = clock();
                num_conflict_updates+=1;
                int conflict_action;
                for (int ac = 0; ac< assign->actions.size();ac++){
                    if (assign->actions[ac].real_action_time>=conflict_time){
                        conflict_action = ac;
                        break;
                    }
                }
                int delay = assign->current_total_delay;
                int cost_increase = assign->cost_increase;
                assign->current_total_delay = prioritized_planning(assign->path,assign->actions, assign->agent,conflict_action);
                if (assign->current_total_delay <0){
                    assignHeap->erase(handleTable[i][j]);
                    delete assign;
                    handleTable[i][j].node_ = NULL;
                    continue;
                }
                assign->cost_increase = assign->current_total_delay - assignments[assign->agent->agent_id].current_total_delay;
                if (ta_option.objective == OBJECTIVE::MAKESPAN){
                    assign->makespan = assign->actions[assign->actions.size()-2].real_action_time;
                    assign->cost_increase = (assign->makespan - current_makespan) > 0 ? (assign->makespan - current_makespan):0;
                }

                assignHeap->update(assign->heap_handle);

                runtime_update_conflict += clock() - t;
            }


        }
        else {

            if (handleTable[i][assignedTask->task_id].node_ != NULL){
                Assignment* assign = *handleTable[i][assignedTask->task_id];
                if (i!=updatedAgent->agent_id){
                    assignHeap->erase(handleTable[i][assignedTask->task_id]);
                    delete assign;
                }
                handleTable[i][assignedTask->task_id].node_ =NULL;

            }

            for (int j = 0; j < handleTable[i].size(); j++) {

                if (handleTable[i][j].node_ == NULL) {
                    continue;
                }

                Assignment *assign = *handleTable[i][j];
                if (screen >= 4) {
                    cout << "Agent: " << assign->agent->agent_id << endl;
                }


                if (updatedAgent->agent_id == assign->agent->agent_id) {
                    clock_t t = clock();
                    num_task_assign_updates += 1;
                    Assignment *new_assign = insertTask(updatedAgent, assign->new_add_task);
                    if (new_assign->current_total_delay < 0) {
                        assignHeap->erase(handleTable[i][j]);
                        delete assign;
                        delete new_assign;
                        handleTable[i][j].node_ = NULL;
                        continue;
                    } else {
                        new_assign->heap_handle = assign->heap_handle;
                        assert(new_assign->actions.size() > assign->actions.size());
                        assignHeap->update(assign->heap_handle, new_assign);
                        delete assign;
                    }
                    runtime_update_changed_agent += clock() - t;
                } else {

                    int conflict_time = haveConflict(assign->agent->agent_id, assign->path);
                    if (conflict_time < 0)
                        continue;
                    clock_t t = clock();
                    num_conflict_updates += 1;
                    int conflict_action;
                    for (int ac = 0; ac < assign->actions.size(); ac++) {
                        if (assign->actions[ac].real_action_time >= conflict_time) {
                            conflict_action = ac;
                            break;
                        }
                    }
                    int delay = assign->current_total_delay;
                    int cost_increase = assign->cost_increase;
                    assign->current_total_delay = prioritized_planning(assign->path, assign->actions, assign->agent,
                                                                       conflict_action);
                    if (assign->current_total_delay < 0) {
                        assignHeap->erase(handleTable[i][j]);
                        delete assign;
                        handleTable[i][j].node_ = NULL;
                        continue;
                    }
                    assign->cost_increase =
                            assign->current_total_delay - assignments[assign->agent->agent_id].current_total_delay;
                    if (ta_option.objective == OBJECTIVE::MAKESPAN){
                        assign->makespan = assign->actions[assign->actions.size()-2].real_action_time;
                        assign->cost_increase = (assign->makespan - current_makespan) > 0 ? (assign->makespan - current_makespan):0;
                    }
//                Assignment* new_assign =  insertTask(updatedAgent, assign->new_add_task);
//                new_assign->heap_handle = assign->heap_handle;

                    assignHeap->update(assign->heap_handle);

                    runtime_update_conflict += clock() - t;
                }
            }
        }
        if(!assignHeap->empty())
            allAssignmentHeap.update(allAssignmentHandles[i]);


    }

}
