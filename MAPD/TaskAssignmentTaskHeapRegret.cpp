//
// Created by Zhe Chen on 10/6/20.
//

#include "TaskAssignmentTaskHeapRegret.h"

void TaskAssignmentTaskHeapRegret::updateAllAssignmentHeap(Agent* updatedAgent,Task* assignedTask){
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

        auto it1 = assignHeap->ordered_begin();
        Assignment* t1_first = *(it1);
        it1++;
        if (it1!=assignHeap->ordered_end()) {
            Assignment *t1_second = *(it1);
            current_best = t1_second->cost_increase - t1_first->cost_increase;
        }
        else{
            current_best = 0;
        }

        if (ta_option.objective == OBJECTIVE::MAKESPAN) {
            for (int j = 0; j < handleTable[i].size(); j++) {
                Assignment *assign = *handleTable[i][j];
                if (screen >= 4) {
                    cout << "Agent: " << assign->agent->agent_id << endl;
                }


                if (updatedAgent->agent_id == assign->agent->agent_id) {
                    clock_t t = clock();
                    num_task_assign_updates += 1;
                    Assignment *new_assign = insertTask(updatedAgent, assign->new_add_task);
                    new_assign->heap_handle = assign->heap_handle;
                    assert(new_assign->actions.size() > assign->actions.size());
                    assignHeap->update(assign->heap_handle, new_assign);
                    delete assign;

                    runtime_update_changed_agent += clock() - t;
                }
                else{
                    assign->cost_increase = assign->makespan - current_makespan >=0 ? assign->makespan - current_makespan : 0;
                }


                assignHeap->update(handleTable[i][j]);
            }
        }
        else{
            int j = updatedAgent->agent_id;

            Assignment* assign = *handleTable[i][j];
            if(screen>=4 ) {
                cout << "Agent: " << assign->agent->agent_id << endl;
            }


            if (updatedAgent->agent_id == assign->agent->agent_id){
                clock_t t = clock();
                num_task_assign_updates+=1;
                Assignment* new_assign =  insertTask(updatedAgent, assign->new_add_task);
                new_assign->heap_handle = assign->heap_handle;
                assert(new_assign->actions.size() > assign->actions.size());
                assignHeap->update(assign->heap_handle, new_assign);
                delete assign;

                runtime_update_changed_agent += clock() - t;
            }

        }




        int new_best;
        auto it2 = assignHeap->ordered_begin();
        Assignment* t2_first = *(it2);
        it2++;
        if (it2!=assignHeap->ordered_end()) {
            Assignment *t2_second = *(it2);
            new_best = t2_second->cost_increase - t2_first->cost_increase;
        }
        else{
            new_best = 0;
        }

        allAssignmentHeap.update(allAssignmentHandles[i]);


    }

}

