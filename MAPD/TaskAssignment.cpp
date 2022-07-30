//
// Created by Zhe Chen on 10/6/20.
//

#include "TaskAssignment.h"

TaskAssignment::TaskAssignment(AgentLoader* agents, TaskLoader* tasks, MapLoaderCost* map,TA_Options ta_opt, options pl_options, bool real_cost, float pl_time_limit){
    this->agents = agents;
    this->tasks = tasks;
    this->map = map;
    this->no_task_loop = ta_opt.no_task_loop;
    this->ta_option = ta_opt;
    this->pl_option = pl_options;
    this->real_cost = real_cost;
    this->planning_time_limit = pl_time_limit;
    this->constraintTable.init(map->map_size());


    start = std::clock();

};

void TaskAssignment::initializeOneShot(){
    for(Task * t : this->tasks->all_tasks){
        t->heap_handle = unassigned_tasks.push(t);

    }
    initializeAssignments();

}

void TaskAssignment::initializeOnline(){

    initializeAssignments();

}

void TaskAssignment::initializeAssignments(){
    assignments.resize(agents->num_of_agents);
    for(Agent* a : agents->agents ){
        assignments[a->agent_id].actions.push_back(ActionEntry(NULL,ACTION_TYPE::START,a->initial_location,0,0));
        assignments[a->agent_id].actions.push_back(ActionEntry(NULL,ACTION_TYPE::DOCK,a->initial_location,1,0));

        assignments[a->agent_id].agent = a;
        assignments[a->agent_id].ideal_cost = 0;
        constraintTable.dock(assignments[a->agent_id].actions.back().location,a->agent_id,assignments[a->agent_id].actions.back().real_action_time);


    }

}



bool TaskAssignment::assignTasks() {
    if(screen>=1) {
        cout<<"** Start Task Assignment **"<<endl;
    }

    if(screen>=2)
        cout << "Building initial heap" << endl;

    initialize_heaps();

    if(screen>=2)
        cout<<"Building initial assignment heap"<<endl;
    buildAssignmentHeap();
    this->first_build = false;
    if(screen>=2)
        cout<<"Done build initial heap"<<endl;

    while (!unassigned_tasks.empty()){
        if(screen>=2){
            cout <<"Selecting Task from "<< unassigned_tasks.size()<<" tasks."<<endl;
        }

        Assignment* min_cost_assign = get_best_assignment();

        int agent_id = min_cost_assign->agent->agent_id;
        int task_id = min_cost_assign->new_add_task->task_id;
        Task* task = min_cost_assign->new_add_task;
        if(screen>=2){
            cout <<"Task:" << task_id << " Assign to agent: " << agent_id<<endl;
        }
        if(screen>=3){
            printTaskHeap();
        }
        assert(!min_cost_assign->path.empty());


        if(!min_cost_assign->path.empty()){

            assert(assignments[agent_id].path.size()==assignments[agent_id].path_length);
            constraintTable.delete_path(agent_id, assignments[agent_id].path);
            constraintTable.insert_path(agent_id, min_cost_assign->path);
            assignments[agent_id].path = min_cost_assign->path;
            assignments[agent_id].path_length = min_cost_assign->path.size();
        }

        constraintTable.delete_dock(assignments[agent_id].actions.back().location,agent_id);
        constraintTable.dock(min_cost_assign->actions.back().location,agent_id,min_cost_assign->actions.back().real_action_time);


        assignments[agent_id].actions = min_cost_assign->actions;
        assignments[agent_id].ideal_cost = min_cost_assign->ideal_cost;
        assignments[agent_id].current_total_delay = min_cost_assign->current_total_delay;
        assignments[agent_id].assigned_tasks.insert(min_cost_assign->new_add_task);
        assignments[agent_id].makespan = min_cost_assign->makespan;

        current_total_delay = 0;
        current_makespan = 0;
//        if (!assignments[29].path.empty())
//            cout<<assignments[29].path[38].location<<endl;

        for (auto& a : assignments){
            current_total_delay += a.current_total_delay;
            if(a.makespan>current_makespan)
                current_makespan = a.makespan;
            if(screen>=2 && !a.path.empty()){
                int c = haveConflict(a.agent->agent_id,a.path);
                assert(c<0);
            }
        }

        if(screen>=2){
            cout <<"Cost increase:" <<min_cost_assign->cost_increase <<endl;
        }



        unassigned_tasks.erase(min_cost_assign->new_add_task->heap_handle);

        assignedTasks.insert(min_cost_assign->new_add_task);
        delete_top(task_id);
        updateAllAssignmentHeap(agents->agents[agent_id], task);
    }
    if(screen>=1) {
        cout<<"** End Task Assignment **"<<endl;
    }


    runtime = (std::clock() - start);
    return true;
}

bool TaskAssignment::optimize(std::unordered_set<int> awaiting_tasks) {
    improving = true;
    int iteration = 0;
    runtime = (std::clock() - start);
    std::clock_t op_start_time = std::clock() - start;

    if(ta_option.objective == OBJECTIVE::MAKESPAN){
        iteration_log.push_back({iteration,0,runtime/CLOCKS_PER_SEC,current_makespan});

    }
    else {
        iteration_log.push_back({iteration, 0, runtime / CLOCKS_PER_SEC, current_total_delay});
    }
    if(screen>=1){
        cout<<"****** Start Anytime Optimization *******"<<endl;
        cout<<"Iteration: "<<iteration<<", select: ";


        cout<<", runtime: "<< (runtime-op_start_time)/CLOCKS_PER_SEC<<", cost: "<< ((ta_option.objective == OBJECTIVE::MAKESPAN) ? current_makespan : current_total_delay) << endl;
    }
    if(awaiting_tasks.empty()){
        for(auto* t : tasks->all_tasks){
            awaiting_tasks.insert(t->task_id);
        }
    }
    while (iteration< ta_option.max_iteration && (runtime-op_start_time)/CLOCKS_PER_SEC < ta_option.time_limit ){
        if (remove_hist.size() >= awaiting_tasks.size()){
            remove_hist.clear();
        }
        iteration++;

        selectRemoveRepair(awaiting_tasks);

        runtime = (std::clock() - start);

        if(screen>=1){
            cout<<"Iteration: "<<iteration<<", select: ";
            for (auto t: selected){
                cout<<t->task_id<<" ";
            }


            cout<<", runtime: "<< (runtime-op_start_time)/CLOCKS_PER_SEC<<", cost: "<< ((ta_option.objective == OBJECTIVE::MAKESPAN) ? current_makespan : current_total_delay) << endl;
        }
        if(ta_option.objective == OBJECTIVE::MAKESPAN){
            iteration_log.push_back({iteration,(int)selected.size(),runtime/CLOCKS_PER_SEC,current_makespan});

        }
        else {
            iteration_log.push_back({iteration, (int) selected.size(), runtime / CLOCKS_PER_SEC, current_total_delay});
        }

    }

    if(screen>=1) {
        cout << "****** End Anytime Optimization *******" << endl;
    }
    return true;

};

bool TaskAssignment::removeFromMax(std::unordered_set<int>& awaiting_tasks) {
    int old_total_delay = current_total_delay;

    int max_id = -1;
    int max_delay = 0;
    int max_makespan = 0;

    for(int a_id = 0; a_id < assignments.size();a_id++){
        bool all_selected = true;
        for(auto* t:assignments[a_id].assigned_tasks){
            if (!remove_hist.count(t) && awaiting_tasks.count(t->task_id)){
                all_selected = false;
                break;
            }
        }
        if ( assignments[a_id].assigned_tasks.size()>0 &&  !all_selected){
            if((ta_option.objective == OBJECTIVE::MAKESPAN && assignments[a_id].makespan>max_makespan)||
               (ta_option.objective != OBJECTIVE::MAKESPAN && assignments[a_id].current_total_delay>max_delay)
                    ) {
                max_delay = assignments[a_id].current_total_delay;
                max_makespan = assignments[a_id].makespan;
                max_id = a_id;
            }
        }
    }
    if(max_id == -1){
        return false;
    }


    auto& max_assign = assignments[max_id];

    vector<Task*> shuffle_list;
    for(auto* t : max_assign.assigned_tasks){
        if(awaiting_tasks.count(t->task_id)){
            shuffle_list.push_back(t);
        }
    }

    selected.clear();
    int count = 0;
    while(selected.size() < ta_option.group_size ){
        if(max_assign.assigned_tasks.size() ==0){
            break;
        }
        int shuffle_id = std::rand() % shuffle_list.size();
        Task* t = shuffle_list[shuffle_id];
        if(!remove_hist.count(t) ) {
            selected.insert(t);
            remove_hist.insert(t);
            max_assign.assigned_tasks.erase(t);
        }
        count +=1;
        if (count >= ta_option.group_size*2){
            break;
        }
    }
    if(screen>=2)
        cout<<"Makespan before remove: "<<current_makespan<<", path size "<<max_assign.makespan <<" , actions "<< max_assign.actions.size()<<endl;



    auto it = max_assign.actions.begin();
    while(it!=max_assign.actions.end()){
        if (selected.count((*it).task)){
            max_assign.actions.erase(it);
        }
        else{
            it++;
        }
    }
    updateActions(max_assign.actions,max_assign.agent,1);
    constraintTable.delete_path(max_assign.agent->agent_id,max_assign.path);
    updateRealPath(&max_assign, max_assign.agent);
    if (max_assign.current_total_delay < 0)
        return false;
    constraintTable.insert_path(max_assign.agent->agent_id,max_assign.path);
    constraintTable.delete_dock(max_assign.actions.back().location,max_assign.agent->agent_id);
    constraintTable.dock(max_assign.actions.back().location,max_assign.agent->agent_id,max_assign.actions.back().real_action_time);
    max_assign.path_length = max_assign.path.size();

    current_total_delay = 0;
    current_makespan = 0;
    for(auto& assign: assignments){
        current_total_delay += assign.current_total_delay;
        if(assign.makespan>=current_makespan){
            current_makespan =assign.makespan;
        }
    }
    if(screen>=2)
        cout<<"Makespan after remove: "<<current_makespan<<", path size "<<max_assign.makespan<<" , actions"<< assignments[max_id].actions.size()<<endl;
    return true;


}
bool TaskAssignment::removeRandom(std::unordered_set<int>& awaiting_tasks) {
    if(screen>=2)
        cout<<"Makespan before remove: "<<current_makespan<<", Delay before remove "<< current_total_delay<<endl;
    vector<int> shuffle_tasks;
    for (int t_id : awaiting_tasks){
        shuffle_tasks.push_back(t_id);
    }
    std::random_shuffle ( shuffle_tasks.begin(), shuffle_tasks.end() );
    selected.clear();

    for(int i =0; i<ta_option.group_size;i++){
        if(i >= shuffle_tasks.size())
            continue;
        assert(tasks->all_tasks_vec[shuffle_tasks[i]]->task_id == shuffle_tasks[i]);
        selected.insert(tasks->all_tasks_vec[shuffle_tasks[i]]);
    }
    current_total_delay = 0;
    current_makespan = 0;
    for(auto& assign: assignments){
        bool removed = false;
        for(Task* t: selected){
            if(assign.assigned_tasks.count(t)) {
                assign.assigned_tasks.erase(t);
                removed=true;
            }
        }

        if(removed) {
            auto it = assign.actions.begin();
            while (it != assign.actions.end()) {
                if (selected.count((*it).task)) {
                    assign.actions.erase(it);
                } else {
                    it++;
                }
            }
        }
        if (removed) {
            updateActions(assign.actions, assign.agent, 1);
            constraintTable.delete_path(assign.agent->agent_id,assign.path);
            updateRealPath(&assign, assign.agent);
            if (assign.current_total_delay < 0)
                return false;

            constraintTable.insert_path(assign.agent->agent_id,assign.path);
            constraintTable.delete_dock(assign.actions.back().location,assign.agent->agent_id);
            constraintTable.dock(assign.actions.back().location,assign.agent->agent_id,assign.actions.back().real_action_time);
            assign.path_length = assign.path.size();
        }
        current_total_delay += assign.current_total_delay;
        if (assign.makespan > current_makespan) {
            current_makespan = assign.makespan;
        }
    }
    if(screen>=2)
        cout<<"Makespan after remove: "<<current_makespan<<", Delay after remove "<< current_total_delay<<endl;

    return true;

}

bool TaskAssignment::removeMultiMax(std::unordered_set<int>& awaiting_tasks) {
    if(screen>=2)
        cout<<"Makespan before remove: "<<current_makespan<<", Delay before remove "<< current_total_delay<<endl;
    vector<Assignment> sorted_assigns = assignments;
    if(ta_option.objective == OBJECTIVE::MAKESPAN){
        std::sort(sorted_assigns.begin(),sorted_assigns.end(),Assignment::compare_assignment_span);
    }
    else{
        std::sort(sorted_assigns.begin(),sorted_assigns.end(),Assignment::compare_assignment_delay);

    }

    selected.clear();
    std::unordered_set<int> selected_agents;
    for(int i =0; i<ta_option.group_size;i++){
        vector<Task*> shuffle_list;
        if (i>= sorted_assigns.size())
            continue;
        for(Task* t: sorted_assigns[i].assigned_tasks){
            if(awaiting_tasks.count(t->task_id)){
                shuffle_list.push_back(t);
            }
        }
        if(shuffle_list.empty())
            continue;
        int s_id = std::rand() % shuffle_list.size();
        Task* t = shuffle_list[s_id];
        selected.insert(t);
        selected_agents.insert(sorted_assigns[i].agent->agent_id);
    }
    current_total_delay = 0;
    current_makespan = 0;
    for(auto& assign: assignments){
        bool removed = false;
        if(selected_agents.count(assign.agent->agent_id)) {
            for(Task* t: selected){
                if(assign.assigned_tasks.count(t)) {
                    assign.assigned_tasks.erase(t);
                    removed=true;
                }
            }


            auto it = assign.actions.begin();
            while (it != assign.actions.end()) {
                if (selected.count((*it).task)) {
                    assign.actions.erase(it);
                    removed = true;
                } else {
                    it++;
                }
            }
        }
        if (removed) {
            updateActions(assign.actions, assign.agent, 1);
            constraintTable.delete_path(assign.agent->agent_id,assign.path);
            updateRealPath(&assign, assign.agent);
            if (assign.current_total_delay < 0)
                return false;
            constraintTable.insert_path(assign.agent->agent_id,assign.path);
            constraintTable.delete_dock(assign.actions.back().location,assign.agent->agent_id);
            constraintTable.dock(assign.actions.back().location,assign.agent->agent_id,assign.actions.back().real_action_time);
            assign.path_length = assign.path.size();
        }
        current_total_delay += assign.current_total_delay;
        if (assign.makespan > current_makespan) {
            current_makespan = assign.makespan;
        }
    }
    if(screen>=2)
        cout<<"Makespan after remove: "<<current_makespan<<", Delay after remove "<< current_total_delay<<endl;
    return true;


}


bool TaskAssignment::selectRemoveRepair(std::unordered_set<int>& awaiting_tasks) {


    int old_total_delay = current_total_delay;
    int old_makespan = current_makespan;
    ConstraintTable old_constraint_table = constraintTable;
    vector<Assignment> old_assigns = assignments;

    bool remove_success = true;
    if (ta_option.destory_method == DESTORY::FROM_MAX)
        remove_success = removeFromMax(awaiting_tasks);
    else if (ta_option.destory_method == DESTORY::RANDOM)
        remove_success = removeRandom(awaiting_tasks);
    else if (ta_option.destory_method == DESTORY::MULTI_MAX)
        remove_success = removeMultiMax(awaiting_tasks);

    if(remove_success) {
        if (screen >= 3)
            cout << "Add to unassigned_tasks" << endl;
        unassigned_tasks.clear();
        for (auto *t:selected) {
            t->heap_handle = unassigned_tasks.push(t);
        }

        if (screen >= 3)
            cout << "Start assignment" << endl;

        assignTasks();

        if (screen >= 2)
            cout << "Makespan after re-assignment: " << current_makespan << ", Delay after re-assignment: "
                 << current_total_delay << endl;

    }
    if(!remove_success || (ta_option.objective != OBJECTIVE::MAKESPAN && current_total_delay > old_total_delay) ||
        (ta_option.objective == OBJECTIVE::MAKESPAN && current_makespan > old_makespan)
    ){
        assignments = old_assigns;
        current_total_delay = old_total_delay;
        current_makespan = old_makespan;
        constraintTable = old_constraint_table;

        if(screen>=2)
            cout<<"Revert to old assignment "<<endl;
    }
    return true;


};



int TaskAssignment::haveConflict(int agent_id, vector<PathEntry>& path){
    for (int i = 1 + (start_timestep>=0?start_timestep:0); i<path.size();i++){
        if (constraintTable.is_path_constrained(agent_id, path[i].location, path[i-1].location,i)) {
            return i;
        }
    }
    return -1;

}

int TaskAssignment::prioritized_planning(vector<PathEntry>& path, vector<ActionEntry>& actions, Agent* a, int next_action){
    clock_t t = clock();
    num_of_pp +=1;

    int start_time = 0;
    int start_loc = a->initial_location;
    int total_delay = 0;
    int nodes_expanded = 0;

    if (next_action > 1){
        assert(next_action > assignments[a->agent_id].start_action);
        assert(!path.empty());
        start_time = actions[next_action-1].real_action_time;
        total_delay = actions[next_action-1].real_current_total_delay;
        assert(start_time >= start_timestep);
//        if(start_time == start_timestep){
//            start_time++; //start_timestep is the time_step previous to current timestep. next_action-1 is a past start action. need to make sure next_action is excutated after start_timestep
//        }
        if(start_time+1>path.size()){
            path.resize(start_time+1,path.back().location);
        }
        else{
            path.resize(start_time+1);
        }
        start_loc = path.back().location;
        assert(start_time == path.size()-1);
    }
    else {
        if(start_timestep>0){
            if(path.empty())
                path.resize(start_timestep+1,PathEntry(actions.front().location));
            else {
                path.resize(start_timestep+1,path.back().location);

            }
            assert(start_timestep == actions[assignments[a->agent_id].start_action].real_action_time);
            start_loc = path[start_timestep].location;
            start_time = start_timestep;
            total_delay = actions[assignments[a->agent_id].start_action].real_current_total_delay;
            assert(start_loc == actions[assignments[a->agent_id].start_action].location);
            next_action = assignments[a->agent_id].start_action+1;

        }
        else{
            path.clear();
        }
    }



    for (int i = next_action; i<actions.size();i++) {

        int start ;
        if(i == next_action){
            start = start_loc;
        }
        else{
            start = actions[i - 1].location;
        }
        int goal = actions[i].location;
        int min_end_time = actions[i].ideal_action_time;
        if(start_time == start_timestep && min_end_time <= start_timestep){
            min_end_time = start_timestep+1;//start_timestep is a past previous time_step.
        }
        if (start == goal && start_time>=min_end_time) {
            if ( actions[i].action_type == ACTION_TYPE::DROP_OFF && start_time > actions[i].task->ideal_end_time){
                total_delay += start_time - actions[i].task->ideal_end_time;
            }

            actions[i].real_current_total_delay = total_delay;
            actions[i].real_action_time = start_time;
            continue;
        }
        SinglePlanning planner(*map,a->agent_id,start,goal,start_time,min_end_time,1.0,0,this->pl_option,this->constraintTable);
        bool success = planner.search(actions[i].action_type == ACTION_TYPE::DOCK);
//        assert(success);
        if (!success){
            return -1;
        }
        if(screen>=5){
            cout<<"Nodes: "<<planner.LL_num_expanded<<endl;
        }
        nodes_expanded += planner.LL_num_expanded;

        if ( actions[i].action_type == ACTION_TYPE::DROP_OFF ){
            total_delay += planner.path.back().timeStep - actions[i].task->ideal_end_time;
        }

        actions[i].real_current_total_delay = total_delay;

        actions[i].real_action_time = planner.path.back().timeStep;
        start_time = planner.path.back().timeStep;
        if (!path.empty())
            path.pop_back();//kick the current start location from existing path's end.
        path.insert( path.end(), planner.path.begin(), planner.path.end() );
        assert(start_time == path.size()-1);
        assert(total_delay>=actions[i].current_total_delay);


    }
    if(screen>=5){
        cout<<"PP done with nodes: "<< nodes_expanded<<endl;
    }
    runtime_pp += clock() - t;
    assert(total_delay>=actions.back().current_total_delay);
    return total_delay;
}

Assignment* TaskAssignment::insertTask(Agent* a,Task* task, bool real_cost){
    Assignment *min_cost_assign = new Assignment();
    int min_cost_increase = INT_MAX/2;
    int start_action = assignments[a->agent_id].start_action+1;


    for (int pick = start_action; pick < assignments[a->agent_id].actions.size(); pick++ ) {
        for (int drop = pick + 1; drop < assignments[a->agent_id].actions.size() + 1; drop++) {


            vector<ActionEntry> temp_actions = assignments[a->agent_id].actions;

            temp_actions.insert(temp_actions.begin() + pick,
                                ActionEntry(task, ACTION_TYPE::PICK_UP, task->initial_location,
                                            task->initial_time));
            temp_actions.insert(temp_actions.begin() + drop,
                                ActionEntry(task, ACTION_TYPE::DROP_OFF, task->goal_location, task->initial_time));

//            if(pick >= assignments[a->agent_id].actions.size()){
//                temp_actions.push_back(ActionEntry(NULL,ACTION_TYPE::DOCK,a->initial_location,1,0));
//            }
            assert(temp_actions.back().action_type==ACTION_TYPE::DOCK);

            if (!updateActions(temp_actions, a, pick, drop)) {
                if (screen >= 5) {
                    cout << "over size" << endl;
                }
                continue;
            }
            vector<PathEntry> path;
            int total_delay;
            int cost_increase;
            total_delay = temp_actions.back().current_total_delay;
            cost_increase = total_delay - assignments[a->agent_id].current_total_delay;
            if (ta_option.objective == OBJECTIVE::MAKESPAN)
                cost_increase = (temp_actions[temp_actions.size() - 2].ideal_action_time - current_makespan) > 0?temp_actions[temp_actions.size() - 2].ideal_action_time - current_makespan:0 ;
//            if (cost_increase < min_cost_increase && real_cost){
//                total_delay = prioritized_planning(path,temp_actions, a);
//                if (total_delay < 0){
//                    continue;
//                }
//                cost_increase = total_delay - assignments[a->agent_id].current_total_delay;
//            }


            if (screen > 5) {
                cout << "Actions length: " << temp_actions.size() << ",cost_increase: " << cost_increase
                     << ", min increase" << min_cost_increase << endl;
            }
            if (cost_increase < min_cost_increase|| (cost_increase == min_cost_increase && temp_actions.back().ideal_action_time<assignments[a->agent_id].ideal_cost)) {

                min_cost_increase = cost_increase;
                min_cost_assign->cost_increase = min_cost_increase;
                min_cost_assign->actions = temp_actions;
                min_cost_assign->ideal_cost = temp_actions.back().ideal_action_time;
                min_cost_assign->current_total_delay = total_delay;
                min_cost_assign->path = path;
                min_cost_assign->makespan = temp_actions[temp_actions.size() - 2].ideal_action_time;
                min_cost_assign->insert_at = pick;


                if (screen > 5) {
                    cout << "assign to: " << a->agent_id << ",cost_increase: " << cost_increase << endl;
                }
                if (min_cost_increase == 0)
                    break;

            }

        }
        if (min_cost_increase == 0)
            break;
    }

    min_cost_assign->agent = a;
    min_cost_assign->new_add_task = task;
    min_cost_assign->path = assignments[a->agent_id].path;
    updateRealPath(min_cost_assign,a,min_cost_assign->insert_at);
    return min_cost_assign;
}

void TaskAssignment::buildAssignmentHeap(){

    for (Task* task : unassigned_tasks){
        if(screen>=4)
            cout<<"Build heap for Task: "<<task->task_id<<endl;
        AssignmentHeap* new_assignment_heap  = new AssignmentHeap();
        for(Agent* a : agents->agents ) {
            if(screen>=5)
                cout<<"Build heap for Agent: "<<a->agent_id<<endl;
            Assignment* min_cost_assign =  insertTask(a, task,ta_option.real_cost_insert);
            if (min_cost_assign->current_total_delay>=0){
                min_cost_assign->heap_handle = new_assignment_heap->push(min_cost_assign);
                handleTable[task->task_id][a->agent_id] = min_cost_assign->heap_handle;
            }
            else{
                handleTable[task->task_id][a->agent_id].node_=NULL;
                delete min_cost_assign;
            }
//            if (min_cost_assign->cost_increase == 0)
//                break;
        }
        logAssignmentHandle(task,NULL, new_assignment_heap);
    }
}

bool TaskAssignment::isValidActions(vector<ActionEntry> &actions, Agent* a) {
    for(int i = 1; i < actions.size(); i++ ){
        if (actions[i].current_load > a->capacity)
            return false;
    }
    return true;
}



bool TaskAssignment::updateActions(vector<ActionEntry> &actions, Agent* agent, int pick, int drop) {
    if (drop == -1){
        drop = actions.size()-1;
    }
    int temp_load = 0;
    int action_time = 0;
    int total_delay = 0;
    if(pick > 0){
        temp_load = actions[pick-1].current_load;
        action_time = actions[pick-1].ideal_action_time;
        total_delay = actions[pick-1].current_total_delay;
    }

    for (int i = pick; i < actions.size(); i++ ){
        ActionEntry& a = actions[i];

        int temp_action_time = map->getDistance(actions[i-1].location,actions[i].location) + action_time;
        if (actions[i].action_type == ACTION_TYPE::PICK_UP && temp_action_time < actions[i].release_time ){
            temp_action_time = actions[i].release_time;
        }
        actions[i].ideal_action_time = temp_action_time;
        action_time = temp_action_time;


        if(a.action_type==ACTION_TYPE::DROP_OFF){
            total_delay = total_delay + (a.ideal_action_time - a.task->ideal_end_time);
            a.current_total_delay = total_delay;
        }
        else{
            a.current_total_delay = total_delay;
        }

        if(i > drop)
            continue;

        if(a.action_type == ACTION_TYPE::START || a.action_type == ACTION_TYPE::DOCK)
            a.current_load = temp_load;
        else if(a.action_type == ACTION_TYPE::PICK_UP){
            temp_load++;
            a.current_load = temp_load;
        }
        else if(a.action_type == ACTION_TYPE::DROP_OFF){
            temp_load--;
            a.current_load = temp_load;
        }
        if(a.current_load > agent->capacity)
            return false;
    }
    return true;
}


void TaskAssignment::printAssignments(){
    for (auto& a: assignments){
        cout<<"Agent: "<< a.agent->agent_id<<", Cost: "<<a.ideal_cost<<", Capacity: "<<a.agent->capacity<< ", Task amount: "<<a.assigned_tasks.size()<<", Actions:";
        a.print();
    }
}

int TaskAssignment::get_num_agents_with_tasks(){
    int count = 0;
    for (auto& a: assignments){
        if (a.actions.size()>1)
            count++;
    }
    return count;
}


