//
// Created by Zhe Chen on 10/6/20.
//
#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>
#include "Agent.h"
#include "agents_loader.h"
#include "Task.h"
#include "TaskAssignment.h"
#include "ICBSSearch.h"
#include "CBSHOnline.h"
#include "PathPlanning.h"
#include "map_loader_with_cost.h"
#include "TaskAssignmentPP.h"
#include "TaskAssignmentRegret.h"
#include "TaskAssignmentTaskHeap.h"
#include "TaskAssignmentTaskHeapRegret.h"
#include "TaskAssignmentRegretTask.h"
#include "TaskAssignmentPPTask.h"
#include "online_simu.h"
int map_cols=0;
int screen = 0;

int main(int argc, char** argv){
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")
            ("map,m", po::value<std::string>()->required(), "input file for map")
            ("agents,a", po::value<std::string>()->required(), "initial locations for agents")
            ("tasks,t", po::value<std::string>()->required(), "a set of tasks")
            ("objective", po::value<std::string>()->default_value("total-travel-time"), "the objective of the task assignment")
            ("screen", po::value<int>()->default_value(0), "screen option (0: none; 1: results; 2:all)")
            ("output,o", po::value<std::string>()->required(), "output file for schedule")
            ("solver,s", po::value<std::string>()->required(), "solvers (CBS, ICBS, CBSH, CBSH-CR, CBSH-R, CBSH-RM, CBSH-GR")
            ("agentNum,k", po::value<int>()->default_value(0), "number of agents")
            ("cutoffTime,c", po::value<float>()->default_value(7200), "cutoff time (seconds)")
            ("capacity", po::value<int>()->default_value(1), "cutoff time (seconds)")
            ("seed,d", po::value<int>()->default_value(0), "random seed")
            ("corridor2", po::value<bool>(), "reason about 2-way branching corridor conflicts")
            ("target", po::value<bool>(), "reason about target conflict")
            ("kDelay", po::value<int>()->default_value(0), "generate k-robust plan")
            ("window,w", po::value<int>()->default_value(0), "The window size of windowed cbs")
            ("RM-4way", po::value<int>()->default_value(1), "0, do not do 4 way split. 1, do 4 way only necessary.2 Always do 4 way splitting for RM, other wise only 4 way when necessary")
            ("f_w", po::value<float>()->default_value(1), "suboptimal bound")
            ("statistic","print statistic data")
            ("only-assignment","only do task assignment")
            ("multi-label","search for multiple goals")
            ("no-task-loop","do not loop over all unassigned tasks")
            ("real-cost","use conflict free cost")
            ("only-update-top","only update top assign")
            ("real-cost-insert","use conflict free cost")
            ("task","marginal cost on task")
            ("regret","regret based")
            ("regret-task","regret on task not agent")
            ("anytime","continue optimze until time-limit")
            ("group-size", po::value<int>()->default_value(1), "anytime group_size")
            ("max-iteration", po::value<int>()->default_value(10000), "max_iteration")
            ("anytime-log", po::value<std::string>()->default_value(""), "output file for anytime log")
            ("destory-method", po::value<std::string>()->default_value("destory-max"), "destory-max or random")
            ("online","online mode")
            ("kiva","load kiva map and tasks");


    po::variables_map temp;
    po::store(po::parse_command_line(argc, argv, desc), temp);
    const po::variables_map vm = temp;
    screen = vm["screen"].as<int>();
    float time_limit = vm["cutoffTime"].as<float>();
    string task = vm["tasks"].as<string>();
    string agent = vm["agents"].as<string>();
    string map = vm["map"].as<string>();
    string objective = vm["objective"].as<string>();
    string output;
    if (vm.count("output")){
        output = vm["output"].as<string>();
    }
    else
        output = "";
    int capacity = vm["capacity"].as<int>();
    bool multi_label = vm.count("multi-label");
    bool real_cost = vm.count("real-cost");
    bool regret = vm.count("regret");
    bool regret_task = vm.count("regret-task");
    srand(vm["seed"].as<int>());



    options options1;
    options1.multi_label = multi_label;
    options1.window_size = vm["window"].as<int>();
    options1.RM4way = vm["RM-4way"].as<int>();
    options1.f_w = vm["f_w"].as<float>();
    options1.ignore_target = true;
    constraint_strategy s;
    if (vm["solver"].as<string>() == "ICBS")
        s = constraint_strategy::ICBS;
    else if (vm["solver"].as<string>() == "CBS")
        s = constraint_strategy::CBS;
    else if (vm["solver"].as<string>() == "CBSH")
        s = constraint_strategy::CBSH;
    else if (vm["solver"].as<string>() == "CBSH-CR")
        s = constraint_strategy::CBSH_CR;
    else if (vm["solver"].as<string>() == "CBSH-R")
        s = constraint_strategy::CBSH_R;
    else if (vm["solver"].as<string>() == "CBSH-RM")
        s = constraint_strategy::CBSH_RM;
    else if (vm["solver"].as<string>() == "CBSH-GR")
        s = constraint_strategy::CBSH_GR;
    else if (vm["solver"].as<string>() == "PBS")
        s = constraint_strategy::PBS;
    else if (vm["solver"].as<string>() == "PP")
        s = constraint_strategy::PP;
    else if (vm["solver"].as<string>() == "REGRET")
        s = constraint_strategy::REGRET;
    else
    {
        std::cout <<"WRONG SOLVER NAME! :" <<vm["solver"].as<string>()<< std::endl;
        return -1;
    }
    options1.s = s;
    if(multi_label && options1.window_size==0){
        cout<<"Window size must larger than 1 when using multi-label a*"<<endl;
        exit(1);
    }

    MapLoaderCost* mapLoader;
    AgentLoader* agentLoader;
    TaskLoader* taskLoader;
    if (vm.count("kiva")){
        mapLoader = new MapLoaderCost();
        agentLoader = new AgentLoader();
        taskLoader = new TaskLoader();

        mapLoader->loadKiva(map);
        agentLoader->loadKiva(agent,capacity,*mapLoader);
        taskLoader->loadKiva(task,*mapLoader);
    }
    else{
        mapLoader = new MapLoaderCost(map);
        agentLoader = new AgentLoader(agent,*mapLoader);
        taskLoader = new TaskLoader(task,*mapLoader);
    }

    if(screen>=1){
        cout<<"load map done"<<endl;
    }
    map_cols = mapLoader->cols;


    if(screen>=1){
        cout<<"load task and agents done"<<endl;
        agentLoader->print();
        taskLoader->printTasks();
    }

    TA_Options ta_options;
    if(objective == "total-travel-time"){
        ta_options.objective = OBJECTIVE::TOTAL_TRAVEL_COST;
    }
    else if(objective == "total-travel-delay"){
        ta_options.objective = OBJECTIVE::TOTAL_TRAVEL_DELAY;
    }
    else if(objective == "makespan"){
        ta_options.objective = OBJECTIVE::MAKESPAN;
    }
    else{
        cout<<"Unknown objective: "<<  vm["objective"].as<string>()<<endl;
        exit(1);
    }
    ta_options.max_iteration = vm["max-iteration"].as<int>();
    ta_options.group_size = vm["group-size"].as<int>();
    ta_options.time_limit = vm["cutoffTime"].as<float>();

    if (vm["destory-method"].as<string>() == "destory-max"){
        ta_options.destory_method = DESTORY::FROM_MAX;
    }
    else if (vm["destory-method"].as<string>() == "random"){
        ta_options.destory_method = DESTORY::RANDOM;
    }
    else if (vm["destory-method"].as<string>() == "multi-max"){
        ta_options.destory_method = DESTORY::MULTI_MAX;
    }
    else{
        cout<<"Invalid destory method"<<endl;
        exit(2);
    }



    if(vm.count("no-task-loop")){
        ta_options.no_task_loop = true;
    }
    if(vm.count("only-update-top")){
        ta_options.only_top = true;
    }
    if(vm.count("real-cost-insert")){
        ta_options.real_cost_insert=true;
    }

    TaskAssignment* taskAssignment;

    if (s == constraint_strategy::PP && vm.count("task")){
        cout<<"PP Task"<<endl;
        taskAssignment = new TaskAssignmentPPTask(agentLoader,taskLoader,mapLoader, ta_options, options1,real_cost,time_limit);
    }

    else if (s == constraint_strategy::PP && regret_task){
        cout<<"PP Regret Task"<<endl;
        taskAssignment = new TaskAssignmentRegretTask(agentLoader,taskLoader,mapLoader, ta_options, options1,real_cost,time_limit);
    }
    else if (s == constraint_strategy::PP && !regret){
        cout<<"PP"<<endl;

        taskAssignment = new TaskAssignmentPP(agentLoader,taskLoader,mapLoader, ta_options, options1,real_cost,time_limit);
    }
    else if (s == constraint_strategy::PP && regret){
        cout<<"PP Regret"<<endl;
        taskAssignment = new TaskAssignmentRegret(agentLoader,taskLoader,mapLoader, ta_options, options1,real_cost,time_limit);
    }
    else if (regret){
        cout<<"Decoupled Regret"<<endl;
        taskAssignment = new TaskAssignmentTaskHeapRegret(agentLoader,taskLoader,mapLoader, ta_options, options1,real_cost,time_limit);
    }
    else if (!regret){
        cout<<"Decoupled"<<endl;
        taskAssignment = new TaskAssignmentTaskHeap(agentLoader,taskLoader,mapLoader, ta_options, options1,real_cost,time_limit);
    }


    if (vm.count("online")){
        //online mode
        OnlineSimu online(taskAssignment, taskLoader, agentLoader, mapLoader);
        online.simulate(vm.count("anytime"));

        if (screen>=1){
            taskAssignment->printAssignments();
            taskAssignment->printPath();
        }

        int ideal_cost = 0;
        int ideal_delay = 0;
        int total_cost = 0;
        int total_delay = 0;
        for (Assignment& assign: taskAssignment->assignments){
            ideal_cost += assign.ideal_cost;
            ideal_delay += assign.actions.back().current_total_delay;
            if (assign.path.size()!= 0)
                total_cost += assign.path.size()-1;
            total_delay +=assign.current_total_delay;
        }

        if (screen >=1)
        cout <<"1"<<","<<agentLoader->num_of_agents<<
              ","<<taskLoader->num_of_tasks<<
              ","<< ideal_cost <<","<<total_cost << "," << ideal_delay <<","<< total_delay << ","
              <<taskAssignment->current_makespan<<","<<taskAssignment->current_makespan<<","
              << taskAssignment->get_num_agents_with_tasks()<< "," << online.runtime / CLOCKS_PER_SEC
              << "," << taskAssignment->runtime_pp / CLOCKS_PER_SEC
              << "," << taskAssignment->runtime_update_conflict / CLOCKS_PER_SEC <<
              "," << taskAssignment->runtime_update_changed_agent / CLOCKS_PER_SEC<<
              ","<< taskAssignment->num_of_pp <<
              ","<< taskAssignment->num_conflict_updates<<
              ","<< taskAssignment->num_task_assign_updates<<
              "," <<endl;

        if (output!="") {

            ofstream stats;
            stats.open(output, ios::app);
            stats <<"1"<<","<<agentLoader->num_of_agents<<
                  ","<<taskLoader->num_of_tasks<<
                  ","<< ideal_cost <<","<<total_cost << "," << ideal_delay <<","<< total_delay << ","
                  <<taskAssignment->current_makespan<<","<<taskAssignment->current_makespan<<","
                  << taskAssignment->get_num_agents_with_tasks()<< "," << taskAssignment->runtime / CLOCKS_PER_SEC
                  << "," << taskAssignment->runtime_pp / CLOCKS_PER_SEC
                  << "," << taskAssignment->runtime_update_conflict / CLOCKS_PER_SEC <<
                  "," << taskAssignment->runtime_update_changed_agent / CLOCKS_PER_SEC<<
                  ","<< taskAssignment->num_of_pp <<
                  ","<< taskAssignment->num_conflict_updates<<
                  ","<< taskAssignment->num_task_assign_updates<<
                  "," <<endl;
            stats.close();
        }
        if(vm.count("anytime")){
            if(vm["anytime-log"].as<string>() != ""){
                ofstream stats;
                stats.open(vm["anytime-log"].as<string>(), ios::trunc);
                stats <<"Iteration,size,runtime,cost"<<endl;
                for(auto log: taskAssignment->iteration_log){
                    stats <<log.iteration<<","<<log.g_size<<","<<log.runtime<<","<<log.final_cost<<endl;
                }
                stats.close();
            }
        }

    }
    else{
        // offline/one-shot mode
        taskAssignment->initializeOneShot();
        taskAssignment->assignTasks();
        if(screen>=1){
            cout<<"TA done"<<endl;
        }

        if(vm.count("anytime")){
            taskAssignment->optimize();

            if(vm["anytime-log"].as<string>() != ""){
                ofstream stats;
                stats.open(vm["anytime-log"].as<string>(), ios::trunc);
                stats <<"Iteration,size,runtime,cost"<<endl;
                for(auto log: taskAssignment->iteration_log){
                    stats <<log.iteration<<","<<log.g_size<<","<<log.runtime<<","<<log.final_cost<<endl;
                }
                stats.close();
            }
        }


        int ideal_cost = 0;
        int ideal_delay = 0;
        int total_cost = 0;
        int total_delay = 0;
        for (Assignment& assign: taskAssignment->assignments){
            ideal_cost += assign.ideal_cost;
            ideal_delay += assign.actions.back().current_total_delay;
            if (assign.path.size()!= 0)
                total_cost += assign.path.size()-1;
            total_delay +=assign.current_total_delay;
        }
        if (screen>=1){
            taskAssignment->printAssignments();
            taskAssignment->printPath();
        }

        if(options1.s == constraint_strategy::PP || vm.count("only-assignment")){

            cout <<"Number agents: "<<agentLoader->num_of_agents<<" Number tasks: "<<taskLoader->num_of_tasks
                 << " Ideal total cost: " << ideal_cost
                 <<", Real cost: "<< total_cost
                 <<", Makespan: "<< taskAssignment->current_makespan
                 <<", Ideal total delay: "<< ideal_delay
                 <<", Real dealy: "<< total_delay<<", TA Runtime: "<<taskAssignment->runtime/CLOCKS_PER_SEC<<
                 ", No of agents with tasks: "<< taskAssignment->get_num_agents_with_tasks()<<endl;
            cout << "runtime_pp: "<< taskAssignment->runtime_pp/CLOCKS_PER_SEC<<
                 " runtime_update_conflict: "<< taskAssignment->runtime_update_conflict/CLOCKS_PER_SEC<<
                 " runtime_update_changed_agent: "<<taskAssignment->runtime_update_changed_agent/CLOCKS_PER_SEC<<
                 " num_conflict_updates: "<< taskAssignment->num_conflict_updates<<
                 " num_task_assign_updates: "<< taskAssignment->num_task_assign_updates<<
                 endl;

            if (output!="") {

                ofstream stats;
                stats.open(output, ios::app);
                stats <<"1"<<","<<agentLoader->num_of_agents<<
                      ","<<taskLoader->num_of_tasks<<
                      ","<< ideal_cost <<","<<total_cost << "," << ideal_delay <<","<< total_delay << ","
                      <<taskAssignment->current_makespan<<","<<taskAssignment->current_makespan<<","
                      << taskAssignment->get_num_agents_with_tasks()<< "," << taskAssignment->runtime / CLOCKS_PER_SEC
                      << "," << taskAssignment->runtime_pp / CLOCKS_PER_SEC
                      << "," << taskAssignment->runtime_update_conflict / CLOCKS_PER_SEC <<
                      "," << taskAssignment->runtime_update_changed_agent / CLOCKS_PER_SEC<<
                      ","<< taskAssignment->num_of_pp <<
                      ","<< taskAssignment->num_conflict_updates<<
                      ","<< taskAssignment->num_task_assign_updates<<
                      "," <<endl;
                stats.close();
            }

        }
        else{
//        if( options1.s == constraint_strategy::PP){
//            options1.s = constraint_strategy::CBS;
//            options1.f_w = 1.1;
//        }
            PathPlanner* planner;
            bool success;

            planner = new PathPlanner(taskAssignment->assignments,time_limit,s,screen, options1,mapLoader);
            success = planner->startPlan();

            if(screen >=1)
                planner->printPath();
            if (success){
                cout<<"Success; ";
            }
            else{
                cout<<"Failed; ";
            }
            int final_cost=0;
            int makespan = 0;
            for(vector<PathEntry>& p : planner->plans){
                final_cost+=p.size()-1;
            }
            for (auto& assign : taskAssignment->assignments){
                if (assign.actions[assign.actions.size()-2].real_action_time > makespan)
                    makespan = assign.actions[assign.actions.size()-2].real_action_time;
            }
            int final_delay = 0;
            for (auto& info : planner->task_info_table){
                final_delay+=info.second.delay;
            }
            cout<<"Ideal total delay: "<<ideal_delay  <<", Final delay: "<< final_delay<<", Ideal cost: "<<ideal_cost << ", Final_cost: " << final_cost << ", Cost increase: "
                <<", ideal Makespan: "<< taskAssignment->current_makespan << ", Makespan"<<makespan<<
                ", TA Runtime: "<<taskAssignment->runtime/CLOCKS_PER_SEC<<", Path Planning Runtime: "<<planner->runtime<<
                ", No of agents with tasks: "<< taskAssignment->get_num_agents_with_tasks()<<endl;

            if (output!="") {
                ofstream stats;
                stats.open(output, ios::app);
                stats <<success<<","<<agentLoader->num_of_agents<<
                      ","<<taskLoader->num_of_tasks<<
                      "," << ideal_cost << "," << final_cost << "," << ideal_delay << "," << final_delay << ","
                      <<taskAssignment->current_makespan<<","<<makespan<<","
                      << taskAssignment->runtime / CLOCKS_PER_SEC << "," << planner->runtime << ","
                      << taskAssignment->get_num_agents_with_tasks() << "," <<planner->num_replan<<","<< endl;
                stats.close();
            }

        }

    }








}
