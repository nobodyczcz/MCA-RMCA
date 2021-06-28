//
// Created by Zhe Chen on 13/5/20.
//

#include "ICBSHSearchPairAnalysis.h"

template <class Map>
void ICBSSearchWithPairedAnalysis<Map>::clear() {
    this->releaseClosedListNodes();
    this->open_list.clear();
    this->focal_list.clear();
    this->paths_found_initially.clear();
//    for(auto path : this->paths){
//        delete path;
//    }
    this->paths.clear();
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

template <class Map>ICBSSearchWithPairedAnalysis<Map>::ICBSSearchWithPairedAnalysis(ICBSSearchWithPairedAnalysis<Map>* engine,int node_limit){
    this->option = engine->option;
    this->option.pairAnalysis = false;
    this->focal_w = engine->focal_w;
    this->node_limit = node_limit;
    this->time_limit = engine->time_limit;
    this->screen = engine->screen;
    this->debug_mode = engine->debug_mode;

    this->cons_strategy = engine->cons_strategy;
    this->rectangleMDD = engine->rectangleMDD;
    this->cardinalRect = engine->cardinalRect;
    this->corridor2 = engine->corridor2;
    this->corridor4 = engine->corridor4;
    this->cardinalCorridorReasoning = engine->cardinalCorridorReasoning;
    this->targetReasoning = engine->targetReasoning;
    this->I_RM = engine->I_RM;

    this->HL_num_expanded = 0;
    this->HL_num_generated = 0;

    this->LL_num_expanded = 0;
    this->LL_num_generated = 0;
    this->num_col = engine->num_col;
    this->al = engine->al;
    this->ml = engine->ml;
    this->num_of_agents = engine->num_of_agents;
    this->map_size = engine->map_size;
    this->solution_found = false;
    this->solution_cost = -1;
    this->kDelay = engine->kDelay;
    this->asymmetry_constraint = false;
    this->ignore_t0 = false;
    this->shortBarrier = false;
    this->search_engines = engine->search_engines;
    this->analysisInstance=true;
}

template <class Map>
bool ICBSSearchWithPairedAnalysis<Map>::pairedAnalysis(ICBSNode* node,int agent1, int agent2) {
    this->clear();

    this->LL_num_generated = 0;
    this->LL_num_expanded = 0;
    this->HL_num_expanded = 0;
    this->HL_num_generated = 0;
    this->solution_found = false;
    this->solution_cost = -1;


    this->dummy_start = new ICBSNode();
    this->dummy_start->agent_id = agent1;

    this->a2_node = new ICBSNode();
    this->a2_node->agent_id = agent2;

    this->dummy_start->parent = this->a2_node;
    this->a2_node->children.push_back(this->dummy_start);

    ICBSNode* curr;
    curr = node;
    while (curr != NULL)
    {
        if(!curr->constraints.empty()){
            if (curr->agent_id == agent1)
            {
                for(auto constraint: curr->constraints){
                    this->dummy_start->constraints.push_back(constraint);
                }
            }
            else if (curr->agent_id == agent2)
            {
                for(auto constraint: curr->constraints){
                    this->a2_node->constraints.push_back(constraint);
                }
            }
        }
        curr = curr->parent;
    }
    this->paths.resize(this->num_of_agents, NULL);
    this->paths_found_initially.resize(this->num_of_agents);

    std::set<int> agents;
    agents.insert(agent1);
    agents.insert(agent2);
    ReservationTable* res_table = new ReservationTable(this->map_size);  // initialized to false
    for (int i = 0; i <this->num_of_agents; i++) {
        //cout << "******************************" << endl;
        //cout << "Agent: " << i << endl;
        if (agents.count(i) == 0) {

            this->paths[i] = new vector<PathEntry>;
            continue;
        }

        this->updateConstraintTable(this->dummy_start,i);
        if(this->screen>=2){
            cout<<"find initial path for " << i <<" length min "<<this->constraintTable.length_min
            <<"length_max"<<this->constraintTable.length_max<<"max"<<this->dummy_start->makespan + 1<<  endl;

        }
        if (this->search_engines[i]->findPath(this->paths_found_initially[i], this->focal_w, this->constraintTable, res_table, this->dummy_start->makespan + 1, 0) == false && this->screen >=2)
            cout << "NO SOLUTION EXISTS";
        if(this->screen>=2){
            cout<<"done " << i << endl;
        }

        this->paths[i] = &(this->paths_found_initially[i]);

        res_table->addPath(i, this->paths[i]);
        this->dummy_start->makespan = max(this->dummy_start->makespan, this->paths_found_initially[i].size() - 1);

        this->LL_num_expanded += this->search_engines[i]->num_expanded;
        this->LL_num_generated += this->search_engines[i]->num_generated;


    }
    delete (res_table);
    this->dummy_start->g_val = 0;
    for (int i = 0; i < this->num_of_agents; i++) {
        if (agents.count(i) == 0) {
            continue;
        }
        this->dummy_start->g_val += this->paths[i]->size() - 1;
    }
    this->dummy_start->h_val = 0;
    this->dummy_start->f_val = this->dummy_start->g_val;

    this->dummy_start->depth = 0;

    this->dummy_start->open_handle = this->open_list.push(this->dummy_start);
    this->dummy_start->focal_handle = this->focal_list.push(this->dummy_start);

    this->HL_num_generated++;
    this->dummy_start->time_generated = this->HL_num_generated;
    this->allNodes_table.push_back(this->dummy_start);
    this->findConflicts(*(this->dummy_start));
    if (this->screen>=2)
    {

        cout << "Pair analysis dummy start conflicts:";
        for (auto &conit : this->dummy_start->unknownConf) {
            cout << "<" << conit->a1<< "," << conit->a2 << ","
                 << "("  << ")" << ","
                 << "("  << ")" << ","
                 << conit->t<< ">; ";

        }
        cout << endl;
    }

    this->min_f_val = this->dummy_start->f_val;
    this->focal_list_threshold =this-> min_f_val * this->focal_w;

    if (this->cons_strategy == constraint_strategy::CBSH_GR || this->cons_strategy == constraint_strategy::CBSH_RM)
        this->mddTable.resize(this->num_of_agents);

    this->start = std::clock();
    if(this->screen>=2){
        cout<<"pair analysis initialize done, start searching" << endl;
    }
    bool result = this->search();
    if(this->screen>=2){
        cout<<"pair analysis done, result: " << result << endl;
    }
    return result;

}




template class ICBSSearchWithPairedAnalysis<MapLoader>;
//template class ICBSSearchWithPairedAnalysis<FlatlandLoader>;

