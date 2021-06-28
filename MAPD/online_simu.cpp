//
// Created by Zhe Chen on 1/12/20.
//

#include "online_simu.h"
#include <stdio.h>

OnlineSimu::OnlineSimu(TaskAssignment* ta,TaskLoader* tl,AgentLoader* al, MapLoader* ml):taskAssignment(ta),taskLoader(tl),agentLoader(al),mapLoader(ml) {

    taskQueue.resize(taskLoader->last_release_time+1);
    agentStatus.resize(agentLoader->num_of_agents);
    for(Task * t : taskLoader->all_tasks){
        taskQueue[t->initial_time].push_back(t);
        unfinished_tasks.insert(t->task_id);
    }
    for(Agent* a: agentLoader->agents){
        agentStatus[a->agent_id].agent = a;
        agentStatus[a->agent_id].currentLoc = a->initial_location;
    }
};


bool OnlineSimu::simulate(bool anytime){
    int timestep = 0;
    initializeAssignments();
    run_start = std::clock();
    while (timestep<=10000){
        if(screen >=1)
            cout<<"Timestep : "<< timestep<<" unfinished: "<<unfinished_tasks.size()<<" awaiting: "<<awaiting_tasks.size()<<" ongoing:"<<ongoing_tasks.size()<<" finished:"<<finished_tasks.size()<<endl;
        if(timestep < taskQueue.size() && taskQueue[timestep].size()>0) {
            setTasks(timestep);
            taskAssignment->start_timestep = timestep-1;
            taskAssignment->assignTasks();
            if (anytime)
                taskAssignment->optimize(awaiting_tasks);
        }

        bool all_done = updateAgentStatus(timestep);
        if(screen>=1 ){
            assert(!haveCollision());
        }
        if (all_done && timestep >= taskQueue.size())
            break;
        timestep++;

    }
    runtime = std::clock()-run_start;
    return true;

}

void OnlineSimu::setTasks(int timestep){
    for (Task* t: taskQueue[timestep]){
        taskAssignment->addToUnassigned(t);
        awaiting_tasks.insert(t->task_id);
    }
    for(Agent* a: agentLoader->agents){



        auto& temp_actions = taskAssignment->assignments[a->agent_id].actions;
        assert(temp_actions.front().real_current_total_delay <10000);

        if(timestep ==0){
            continue;
        }
        else if (temp_actions[agentStatus[a->agent_id].prevAction].action_type==ACTION_TYPE::START){
            temp_actions[agentStatus[a->agent_id].prevAction].location = agentStatus[a->agent_id].currentLoc;
            temp_actions[agentStatus[a->agent_id].prevAction].real_action_time = timestep-1;
            temp_actions[agentStatus[a->agent_id].prevAction].release_time = timestep-1;
            continue;
        }
        temp_actions.insert(temp_actions.begin() + agentStatus[a->agent_id].prevAction+1,
                ActionEntry(NULL, ACTION_TYPE::START, agentStatus[a->agent_id].currentLoc, timestep-1));
        taskAssignment->updateActions(temp_actions,a,agentStatus[a->agent_id].prevAction+1);
        assert(temp_actions[agentStatus[a->agent_id].prevAction+1].action_type == ACTION_TYPE::START);
        temp_actions[agentStatus[a->agent_id].prevAction+1].real_action_time = timestep-1;
        temp_actions[agentStatus[a->agent_id].prevAction+1].real_current_total_delay = temp_actions[agentStatus[a->agent_id].prevAction].real_current_total_delay;
        taskAssignment->assignments[a->agent_id].start_action = agentStatus[a->agent_id].prevAction+1;
        agentStatus[a->agent_id].prevAction++;

    }
}

void OnlineSimu::initializeAssignments(){
    taskAssignment->initializeOnline();

}

bool OnlineSimu::updateAgentStatus(int timestep){
    int out_of_path = 0;
    for(Agent* a: agentLoader->agents){
        vector<ActionEntry>& actions = taskAssignment->assignments[a->agent_id].actions;
        assert(timestep<=actions[agentStatus[a->agent_id].prevAction+1].real_action_time || actions[agentStatus[a->agent_id].prevAction+1].action_type==ACTION_TYPE::DOCK);

        if (timestep ==0){
            assert(agentStatus[a->agent_id].currentLoc == actions.front().location);
            agentStatus[a->agent_id].prevAction++;
            continue;
        }

        if(taskAssignment->assignments[a->agent_id].path.empty()){
            assert(actions.size() == 2);
            continue;
        }
        else if(timestep >= taskAssignment->assignments[a->agent_id].path.size() ){
            assert(actions[agentStatus[a->agent_id].prevAction+1].action_type == ACTION_TYPE::DOCK);
            assert(actions[agentStatus[a->agent_id].prevAction+1].location == agentStatus[a->agent_id].currentLoc);
            out_of_path++;
            continue;
        }


        assert(agentStatus[a->agent_id].currentLoc == taskAssignment->assignments[a->agent_id].path[timestep-1].location);
        int old_loc = agentStatus[a->agent_id].currentLoc;
        agentStatus[a->agent_id].currentLoc = taskAssignment->assignments[a->agent_id].path[timestep].location;
        int diff = abs(agentStatus[a->agent_id].currentLoc - old_loc);
        assert(diff == 0 || diff == 1 || diff == mapLoader->cols);

        while (actions[agentStatus[a->agent_id].prevAction+1].real_action_time == timestep
          && actions[agentStatus[a->agent_id].prevAction+1].action_type!=ACTION_TYPE::DOCK ){
            assert(agentStatus[a->agent_id].currentLoc == actions[agentStatus[a->agent_id].prevAction+1].location);
            if (agentStatus[a->agent_id].currentLoc != actions[agentStatus[a->agent_id].prevAction+1].location){
                printf("Error: time location not sync, timestep: %d , target location: %d, current location: %d",timestep, actions[agentStatus[a->agent_id].prevAction+1].location, agentStatus[a->agent_id].currentLoc);
                exit(1);
            }

            // update current load
            if(actions[agentStatus[a->agent_id].prevAction+1].action_type==ACTION_TYPE::PICK_UP){
                assert(!agentStatus[a->agent_id].currentLoads.count(actions[agentStatus[a->agent_id].prevAction+1].task));
                agentStatus[a->agent_id].currentLoads.insert(actions[agentStatus[a->agent_id].prevAction+1].task);
                assert(!ongoing_tasks.count(actions[agentStatus[a->agent_id].prevAction+1].task->task_id));
                ongoing_tasks.insert(actions[agentStatus[a->agent_id].prevAction+1].task->task_id);
                assert(awaiting_tasks.count(actions[agentStatus[a->agent_id].prevAction+1].task->task_id));
                awaiting_tasks.erase(actions[agentStatus[a->agent_id].prevAction+1].task->task_id);

            }
            else if (actions[agentStatus[a->agent_id].prevAction+1].action_type==ACTION_TYPE::DROP_OFF) {
                agentStatus[a->agent_id].currentLoads.erase(actions[agentStatus[a->agent_id].prevAction+1].task);
                unfinished_tasks.erase(actions[agentStatus[a->agent_id].prevAction+1].task->task_id);
                assert(ongoing_tasks.count(actions[agentStatus[a->agent_id].prevAction+1].task->task_id));
                ongoing_tasks.erase(actions[agentStatus[a->agent_id].prevAction+1].task->task_id);
                finished_tasks.insert(actions[agentStatus[a->agent_id].prevAction+1].task->task_id);

            }

            assert(agentStatus[a->agent_id].currentLoads.size() == actions[agentStatus[a->agent_id].prevAction+1].current_load);
            assert(agentStatus[a->agent_id].currentLoads.size() <= a->capacity);

            //update prev action
            agentStatus[a->agent_id].prevAction++;

        }

        assert(timestep<actions[agentStatus[a->agent_id].prevAction+1].real_action_time || actions[agentStatus[a->agent_id].prevAction+1].action_type==ACTION_TYPE::DOCK);

    }
    if(out_of_path == agentLoader->num_of_agents && timestep > taskQueue.size() ){
        printf("Error: no path for all %d agents. Timestep %d >= %ld",agentLoader->num_of_agents,timestep, taskQueue.size());
        exit(1);
    }
    return unfinished_tasks.empty();
}

bool OnlineSimu::haveCollision(){
    for(int a1=0;a1<agentLoader->num_of_agents;a1++){
        for(int a2=a1+1; a2<agentLoader->num_of_agents;a2++){
            if(agentStatus[a1].currentLoc == agentStatus[a2].currentLoc) {
                cout<<a1<<","<<a2<<" have collision at "<< agentStatus[a1].currentLoc<<endl;
                return true;
            }
        }
    }
    return false;
}


