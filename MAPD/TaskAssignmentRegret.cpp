//
// Created by Zhe Chen on 10/6/20.
//

#include "TaskAssignmentRegret.h"


void TaskAssignmentRegret::updateAllAssignmentHeap(Agent* updatedAgent,Task* assignedTask){
    bool taskheap_have_zero_increase = false;
    if(screen>=3)
        cout<<"Update Task Heap"<<endl;

    for (auto& handlePair : allAssignmentHandles){
        int i = handlePair.first;
        if (handleTable[i].empty()) {
            continue;
        }
        AssignmentHeap* assignHeap = *handlePair.second;
        int current_best;

       if(ta_option.only_top){

            if (ta_option.objective == OBJECTIVE::MAKESPAN){
                for (int j =0;j<assignHeap->size();j++){
                    if (handleTable[i][j].node_ == NULL)
                        continue;
                    Assignment* assign = *handleTable[i][j];
                    assign->cost_increase = assign->makespan - current_makespan >=0 ? assign->makespan - current_makespan : 0;
                    assignHeap->update(handleTable[i][j]);
                }
            }
            if(screen>=4)
                cout<<"Update "<< i <<" "<< updatedAgent->agent_id <<endl;
            if (handleTable[i][updatedAgent->agent_id].node_ != NULL){
                int j = updatedAgent->agent_id;
                Assignment* assign = *handleTable[i][j];
                clock_t t = clock();
                num_task_assign_updates+=1;
                Assignment* new_assign =  insertTask(updatedAgent, assign->new_add_task);
                if (new_assign->current_total_delay <0) {
                    assignHeap->erase(handleTable[i][j]);
                    delete assign;
                    delete new_assign;
                    handleTable[i][j].node_ = NULL;
                    // continue;
                }
                else {
                    new_assign->heap_handle = assign->heap_handle;
                    assert(new_assign->actions.size() > assign->actions.size());
                    assignHeap->update(assign->heap_handle, new_assign);
                    delete assign;
                }
                runtime_update_changed_agent += clock() - t;
            }

            bool stop = false;
            while (!stop){
                if(screen>=4)
                    cout<<"Check top of "<< i <<endl;
                auto it = assignHeap->ordered_begin();
                stop = true;
                for (int count = 0;count<2;count++, it++){
                    if (it == assignHeap->ordered_end()){
                        continue;
                    }
                    Assignment* assign = (*it);
                    int j = assign->agent->agent_id;
                    int conflict_time =  haveConflict(assign->agent->agent_id,assign->path);
                    if (conflict_time<0) {
                        continue;
                    }
                    else{
                        assert(conflict_time>start_timestep);
                        stop = false;
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
                        stop = false;
                        continue;
                    }
                    assign->cost_increase = assign->current_total_delay - assignments[assign->agent->agent_id].current_total_delay;
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
            if(screen>=4)
                    cout<<"Check top of "<< i <<"Done"<<endl;


        }
        else {

            for (int j = 0; j < handleTable[i].size(); j++) {
                if (handleTable[i][j].node_ == NULL) {
                    continue;
                }

                Assignment *assign = *handleTable[i][j];
                if (screen >= 4) {
                    cout << "Agent: " << assign->agent->agent_id << endl;
                }


                if (updatedAgent->agent_id == assign->agent->agent_id ) {
                    clock_t t = clock();
                    num_task_assign_updates += 1;
                    Assignment *new_assign = insertTask(updatedAgent, assign->new_add_task);
                    if (new_assign->current_total_delay < 0) {
                        assignHeap->erase(handleTable[i][j]);
                        delete assign;
                        delete new_assign;
                        handleTable[i][j].node_ = NULL;
                        // continue;
                    } else {
                        new_assign->heap_handle = assign->heap_handle;
                        assert(new_assign->actions.size() > assign->actions.size());
                        assignHeap->update(assign->heap_handle, new_assign);
                        delete assign;
                    }
                    runtime_update_changed_agent += clock() - t;
                }
//                else if (haveConflict(assign->agent->agent_id, assign->path)) {
//                    clock_t t = clock();
//                    num_task_assign_updates += 1;
//                    Assignment *new_assign = insertTask(assign->agent, assign->new_add_task);
//                    if (new_assign->current_total_delay < 0) {
//                        assignHeap->erase(handleTable[i][j]);
//                        delete assign;
//                        delete new_assign;
//                        handleTable[i][j].node_ = NULL;
//                        continue;
//                    } else {
//                        new_assign->heap_handle = assign->heap_handle;
//                        assignHeap->update(assign->heap_handle, new_assign);
//                        delete assign;
//                    }
//                    runtime_update_changed_agent += clock() - t;
//                }
                else {

                    int conflict_time = haveConflict(assign->agent->agent_id, assign->path);
                    if (conflict_time < 0)
                        continue;
                    assert(conflict_time>start_timestep);

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


        allAssignmentHeap.update(allAssignmentHandles[i]);


    }
    if(screen>=3)
        cout<<"Update Task Heap Done"<<endl;

}
