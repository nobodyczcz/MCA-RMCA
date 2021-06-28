//
// Created by Zhe Chen on 10/6/20.
//
#include <vector>

#include "map_loader_with_cost.h"
#include "basic.h"
#include<boost/tokenizer.hpp>
#include <utility>
#include <iostream>
#include <fstream>

extern int map_cols;

#ifndef LIFELONG_MAPF_AGENT_H
#define LIFELONG_MAPF_AGENT_H
using namespace std;
using namespace boost;
class Agent{
public:
    Agent(int id, int initial_loc,int capacity);

    int agent_id;
    int initial_location;
    int capacity;

};

class AgentLoader{
public:
    AgentLoader(const std::string fname, const MapLoaderCost &ml);
    AgentLoader(){};
    void loadKiva(const std::string fname,int capacity, const MapLoaderCost &ml);
    int num_of_agents;
    vector<Agent* > agents;
    ~AgentLoader(){
        for (auto a : agents){
            delete a;
        }
    }
    void print(){
        for(auto a: agents){
            cout<<"Agent: "<< a->agent_id<<", ("<<a->initial_location/map_cols<<","<<a->initial_location%map_cols<<"),"<<a->capacity<<endl;
        }
    }
};
#endif //LIFELONG_MAPF_AGENT_H

