//
// Created by Zhe Chen on 10/6/20.
//
#include "CBSHOnline.h"
template <class Map>
void CBSHOnline<Map>::clear() {
    this->releaseClosedListNodes();
    this->open_list.clear();
    this->focal_list.clear();
    for (size_t i = 0; i < this->search_engines.size(); i++){
        if(this->search_engines[i]!=NULL) {
            delete (this->search_engines[i]);
            this->search_engines[i]=NULL;
        }
    }
    this->focal_w = 1;
    this->search_engines.clear();

    if (!this->mddTable.empty())
    {
        for (int i = 0; i < this->num_of_agents; i++)
        {
            for (auto mdd : this->mddTable[i])
            {
                if (mdd.second != nullptr)
                    delete mdd.second;
            }
        }
        this->mddTable.clear();
    }

}

template<class Map>
CBSHOnline<Map>::CBSHOnline(Map* ml, AgentsLoader& al, double f_w, constraint_strategy c, int time_limit, int screen, int kDlay, options options1)
{
    this->option = options1;
    this->focal_w = f_w;
    this->time_limit = time_limit;
    this->screen = screen;
    if (screen >= 2||options1.debug)
        this->debug_mode = true;


    if (this->debug_mode)
        cout << "[CBS] Initializing CBS" << endl;
    this->cons_strategy = c;
    this->HL_num_expanded = 0;
    this->HL_num_generated = 0;

    this->LL_num_expanded = 0;
    this->LL_num_generated = 0;
    this->num_col = ml->cols;
    this->al = al;
    this->ml = ml;
    this->map_size = ml->rows*ml->cols;
    this->kDelay = kDlay;
    this->asymmetry_constraint = options1.asymmetry_constraint;
    this->ignore_t0 = options1.ignore_t0;
    this->shortBarrier = options1.shortBarrier;
    if(c == constraint_strategy::CBSH_RM) {
        this->rectangleMDD =true;
    }
}

template<class Map>
void CBSHOnline<Map>::initializeSearchEngine() {
//    if(this->HL_num_expanded>=78529) {
//        this->screen = 4;
//        this->debug_mode = true;
//    }
    this->num_of_agents = this->al.num_of_agents;
    this->solution_cost = -1;
    this->solution_found = false;
    this->search_engines = std::vector<SingleAgentICBS<Map>* >(this->num_of_agents);
    if (this->debug_mode)
        cout << "Initializing search engines" << endl;
    this->constraintTable = ConstraintTable();
    for (int i = 0; i < this->num_of_agents; i++) {
        if (this->debug_mode)
            cout << "initializing agent "<< i << endl;

        int init_loc = this->ml->linearize_coordinate((this->al.initial_locations[i]).first,
                                                      (this->al.initial_locations[i]).second);
        vector<int> goals;
        for(int g=0;g<this->al.goal_locations[i].size();g++) {
            goals.push_back( this->ml->linearize_coordinate((this->al.goal_locations[i][g]).first,
                                                          (this->al.goal_locations[i][g]).second));
        }


        this->search_engines[i] = new SingleAgentICBS<Map>(init_loc, goals, this->ml,i, this->al.headings[i],this->kDelay,this->al.min_end_time[i]);
        this->search_engines[i]->lower_bound =0;
    }

    if (this->debug_mode) {
        cout << "Initializing search engines done" << endl;
    }
    if(this->screen>=1){
        this->al.printAgentsInitGoal();
    }
    this->start = std::clock();
}

template<class Map>
void CBSHOnline<Map>::initializeDummyStart() {
    this->dummy_start = new ICBSNode();
    this->dummy_start->agent_id = -1;
    if(this->screen>=3){
        this->al.printAgentsInitGoal();
    }
    if (this->debug_mode)
        cout << "Initializing first solutions" << endl;
    // initialize paths_found_initially
    this->paths.clear();
    this->paths_found_initially.clear();
    this->paths.resize(this->num_of_agents, NULL);
    this->paths_found_initially.resize(this->num_of_agents);
    ReservationTable* res_table = new ReservationTable(this->map_size);  // initialized to false
    for (int i = 0; i < this->num_of_agents; i++) {
        //cout << "******************************" << endl;
        if(this->screen>=3) {
            cout << "Initializing Agent: " << i << endl;
        }
        if(this->al.done[i]){
            this->paths_found_initially[i] = vector<PathEntry>();
            this->paths_found_initially[i].resize(1);
            this->paths[i] = &this->paths_found_initially[i];
            continue;
        }
        if (this->search_engines[i]->findPath(this->paths_found_initially[i], this->focal_w, this->constraintTable, res_table, this->dummy_start->makespan + 1, 0) == false && !this->analysisInstance)
            cout << "NO SOLUTION EXISTS";

        this->paths[i] = &this->paths_found_initially[i];
        /*if (paths[i]->at(2).conflist == NULL) {
            cout << "x" << endl;

        }
        else {
            cout << paths[i]->at(2).conflist->size() << endl;

        }*/
        res_table->addPath(i, this->paths[i]);
        this->dummy_start->makespan = max(this->dummy_start->makespan, this->paths_found_initially[i].size() - 1);

        this->LL_num_expanded += this->search_engines[i]->num_expanded;
        this->LL_num_generated += this->search_engines[i]->num_generated;

    }
    delete (res_table);

    //printPaths();

    if (this->debug_mode)
        cout << "Initializing dummy start" << endl;
    // generate dummy start and update data structures
    this->dummy_start->g_val = 0;
    for (int i = 0; i < this->num_of_agents; i++)
        this->dummy_start->g_val += this->paths[i]->size() - 1;
    this->dummy_start->h_val = 0;
    this->dummy_start->f_val = this->dummy_start->g_val;

    this->dummy_start->depth = 0;

    this->dummy_start->open_handle = this->open_list.push(this->dummy_start);
    this->dummy_start->focal_handle = this->focal_list.push(this->dummy_start);

    this->HL_num_generated++;
    this->dummy_start->time_generated = this->HL_num_generated;
    this->allNodes_table.push_back(this->dummy_start);
    if (this->debug_mode)
        cout << "Find initial conflict" << endl;
    this->findConflicts(*this->dummy_start);
    if (this->debug_mode)
        cout << "Find initial conflict done" << endl;
    this->min_f_val = this->dummy_start->f_val;
    this->focal_list_threshold = this->min_f_val * this->focal_w;
    if (this->debug_mode)
    {
        this->printPaths();

        cout << "Dummy start conflicts:";
        for (auto &conit : this->dummy_start->unknownConf) {
            cout << "<" << conit->a1<< "," << conit->a2 << ","
                 << "("  << ")" << ","
                 << "("  << ")" << ","
                 << conit->t<< ">; ";

        }
        cout << endl;
    }
//     if (cons_strategy == constraint_strategy::CBSH_GR )
    if (this->cons_strategy == constraint_strategy::CBSH_GR || this->cons_strategy == constraint_strategy::CBSH_RM)
        this->mddTable.resize(this->num_of_agents);
    if (this->option.pairAnalysis){
        this->pairAnalysisTable.resize(this->num_of_agents);
    }
    if(this->debug_mode)
        cout << "Initializing done" << endl;

}

template<class Map>
void CBSHOnline<Map>::set_focal_w(float f_w) {
    this->focal_w = f_w;
}



template class CBSHOnline<MapLoader>;

