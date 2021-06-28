//
// Created by Zhe Chen on 10/6/20.
//

#include "PathPlanning.h"
#include "agents_loader.h"

///
/// \param assignments
/// \param cbs
PathPlanner::PathPlanner(vector<Assignment>& assignments,float cut_off, constraint_strategy s,int screen, options option,MapLoaderCost* mapLoader)
    :assignments(assignments),cut_off(cut_off),s(s),screen(screen),mapLoader(mapLoader){
    plans.resize(assignments.size());
    cbs_option = option;
    window_size = option.window_size;
    if (s == constraint_strategy::PBS){
        pbs = true;
    }
    else{
        pbs = false;
    }
};


bool PathPlanner::startPlan() {
    std::clock_t start = std::clock();
    bool all_done = false;
    int timestep = 0;
    int plan_start = 0;
    vector<vector<PathEntry>> current_plan;
    current_plan.resize(assignments.size());

    bool need_replan = true;
    for (auto& assign : assignments){
        assign.current_target_index = 0;
        assign.done = false;
        current_plan[assign.agent->agent_id] = vector<PathEntry>();
        current_plan[assign.agent->agent_id].push_back(PathEntry(assign.actions[0].location));
    }
    int window_count = window_size;
    while (!all_done){
        if (window_count == 0 && window_size>0)
            need_replan = true;
        if (screen>=3){
            cout<<"Time: "<<timestep<<endl;
        }

        for (int agent_id = 0;agent_id < plans.size();agent_id++){
            if (assignments[agent_id].done){
                continue;
            }
            int current_target_index = assignments[agent_id].current_target_index;
            if(current_plan[agent_id][timestep-plan_start].location == assignments[agent_id].actions[current_target_index].location &&
                    timestep >=  assignments[agent_id].actions[current_target_index].ideal_action_time   ){ //if finish an action
                plans[agent_id].push_back(current_plan[agent_id][timestep-plan_start]);
                plans[agent_id].back().timeStep = timestep;
                ActionEntry& action = assignments[agent_id].actions[assignments[agent_id].current_target_index];

                if(action.action_type == ACTION_TYPE::DROP_OFF){
                    task_info_table[action.task] = Task_info();
                    task_info_table[action.task].delay = timestep - action.task->ideal_end_time;
                }
                action.real_action_time = timestep;

                if(assignments[agent_id].current_target_index < assignments[agent_id].actions.size()-1){
                    if(!cbs_option.multi_label)
                    need_replan = true;

                    assignments[agent_id].current_target_index ++;

                }
                else{

                    assignments[agent_id].done = true;
                }
            }
            else {
                plans[agent_id].push_back(current_plan[agent_id][timestep-plan_start]);
                plans[agent_id].back().timeStep = timestep;

            }

        }

        int done = 0;
        for (auto& assign:assignments){
            if(assign.done)
                done++;
        }

        if(done == plans.size())
            all_done = true;

        if(need_replan){
            if (screen >= 2){
                cout<<"Replanning...."<<endl;
                cout<<"Timestep: "<<timestep<<endl;

            }
            AgentsLoader al;

            for(auto& assign:assignments){
                ActionEntry* action;
                PathEntry current;
                if (assign.done) {

                    action = &assign.actions.back();
                    current = plans[assign.agent->agent_id].back();
                    int min_end = action->ideal_action_time - timestep;
                    if (min_end < 0)
                        min_end = 0;
                    al.addAgent(current.location / mapLoader->cols, current.location % mapLoader->cols,
                                action->location / mapLoader->cols, action->location % mapLoader->cols,
                                min_end, assign.done);
                }
                else if(cbs_option.multi_label){

                    current = current_plan[assign.agent->agent_id][timestep - plan_start];
                    vector<pair<int,int>> goals;
                    vector<int> min_ends;
                    int travel_distance = 0;
                    for(int index = assign.current_target_index; index<assign.actions.size();index++){
                        action = &assign.actions[index];
                        //skip  action on same location;
                        if (goals.empty() && action->location == current.location && timestep>= action->ideal_action_time){
                            if (action->action_type == ACTION_TYPE::DROP_OFF) {

                                task_info_table[action->task] = Task_info();
                                task_info_table[action->task].delay = timestep - action->task->ideal_end_time;

                            }
                            action->real_action_time = timestep;

                            assign.current_target_index++;
                            continue;
                        }
                        if(!goals.empty() &&
                        (goals.back().first*mapLoader->cols + goals.back().second) == action->location &&
                        min_ends.back()>=action->ideal_action_time
                        ){
                            continue;
                        }

                        if(!goals.empty())
                            travel_distance+=mapLoader->getDistance((goals.back().first*mapLoader->cols + goals.back().second) ,action->location,-1);
                        else{
                            travel_distance+=mapLoader->getDistance(current.location ,action->location,-1);
                        }
                        int release = action->ideal_action_time - timestep;
                        if (release < 0)
                            release = 0;
                        goals.push_back(make_pair(action->location / mapLoader->cols, action->location % mapLoader->cols));
                        min_ends.push_back(release);
                        if(travel_distance >= window_size){
                            break;
                        }
                    }
                    if (assign.current_target_index >= assign.actions.size() - 1 &&
                        action->location == current.location)
                        assign.done = true;
                    al.addAgent(current.location / mapLoader->cols, current.location % mapLoader->cols, goals, min_ends, assign.done);

                }
                else {
                    current = current_plan[assign.agent->agent_id][timestep - plan_start];
                    action = &assign.actions[assign.current_target_index];


                    while (action->location == current.location && timestep >= action->release_time &&
                           assign.current_target_index < assign.actions.size() - 1) {

                        if (action->action_type == ACTION_TYPE::DROP_OFF) {

                            task_info_table[action->task] = Task_info();
                            task_info_table[action->task].delay = timestep - action->task->ideal_end_time;
                        }
                        action->real_action_time = timestep;

                        assign.current_target_index++;
                        action = &assign.actions[assign.current_target_index];
                    }

                    if (assign.current_target_index >= assign.actions.size() - 1 &&
                        action->location == current.location)
                        assign.done = true;

                    int min_end = action->ideal_action_time - timestep;
                    if (min_end < 0)
                        min_end = 0;
                    al.addAgent(current.location / mapLoader->cols, current.location % mapLoader->cols,
                                action->location / mapLoader->cols, action->location % mapLoader->cols,
                                min_end, assign.done);
                }
            }


            CBSHOnline<MapLoader> cbsh(mapLoader, al, cbs_option.f_w, s, cut_off * CLOCKS_PER_SEC, screen, 0, cbs_option);
            cbsh.initializeSearchEngine();
            cbsh.initializeDummyStart();
            bool success = cbsh.search();
            if (!success) {
                runtime = (std::clock()-start)/CLOCKS_PER_SEC;
                return false;
            }
            current_plan = cbsh.getPaths();

            num_replan++;

            plan_start = timestep;

            need_replan = false;
            window_count = window_size;
        }
        timestep++;
        if(window_size>0)
            window_count--;

    }
    runtime = (std::clock()-start)/CLOCKS_PER_SEC;
    if(!checkResult()){
        printPath();
        cout<<"solution not correct"<<endl;
        exit(1);
    }
    return true;

}

void PathPlanner::printPath(){
    for(int agent_id = 0;agent_id<plans.size();agent_id++){
        cout<<"Agent: "<<agent_id<<" , Cost: "<<plans[agent_id].size()-1 << ", plan: ";
        for (auto& entry : plans[agent_id]){
            cout<<entry.timeStep<<"("<<entry.location/mapLoader->cols<<","<<entry.location%mapLoader->cols<<")->";
        }
        cout<<endl;
    }
}

bool PathPlanner::checkResult(){
    auto& assign = assignments;
    for (int i = 0; i < plans.size();i++){
        int next_action = 1;
        int next_goal = assign[i].actions[next_action].location;
        bool find = false;
        for(int t=0; t < plans[i].size() ; t++){
            if (plans[i][t].location == next_goal){
                if(t< assign[i].actions[next_action].ideal_action_time){
                    continue;
                }
                find = true;
                next_action++;
                if(next_action <assign[i].actions.size()) {
                    next_goal = assign[i].actions[next_action].location;
                    while (assign[i].actions[next_action].location == next_goal && next_action<assign[i].actions.size()-1) {
                        next_action++;
                        next_goal = assign[i].actions[next_action].location;
                    }
                    if(next_action<assign[i].actions.size()) {
                        find = false;
                    }
                }
                else{
                    if(t != plans[i].size()-1) {
                        if(screen>=1) {
                            cout << "reach goal but time" << t << "not end of path of agent" << i << endl;
                        }
                    }
                }
            }
        }
        if (!find && assign[i].actions.size() > 1) {
            cout<<"didn't reach action #"<<next_action<<" "<<next_goal/mapLoader->cols<<","<<next_goal%mapLoader->cols <<" release time: "<<assign[i].actions[next_action].release_time <<" agent "<< i <<endl;
            cout <<"Total actions "<< assign[i].actions.size()<<endl;
            return false;
        }
    }
    for (int i = 0;i<plans.size(); i++){
        for (int j = i+1;j<plans.size();j++){
            for(int t=0;t<min(plans[i].size(),plans[j].size());t++){
                if (plans[i][t].location ==plans[j][t].location ){
                    cout<<"find vertex conflict"<<endl;
                    return false;
                }

                if (t+1 < min(plans[i].size(),plans[j].size()) && plans[i][t].location ==plans[j][t+1].location && plans[i][t+1].location ==plans[j][t+1].location){
                    cout<<"find edge conflict"<<endl;
                    return false;
                }
            }
        }

    }
    return true;
}
