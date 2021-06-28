//
// Created by Zhe Chen on 13/5/20.
//

#pragma once
#include "ICBSSearch.h"
#include "ICBSNode.h"
#include "map_loader.h"
//#include "flat_map_loader.h"

template <typename Map>
class ICBSSearchWithPairedAnalysis : public MultiMapICBSSearch<Map>{
public:
    void clear();
    bool pairedAnalysis(ICBSNode* node,int agent1, int agent2);
    ICBSSearchWithPairedAnalysis(Map * ml, AgentsLoader & al, double f_w, constraint_strategy c, int time_limit, int screen,int kDlay, options options1):
        MultiMapICBSSearch<Map>(ml, al,f_w,c,time_limit,screen,kDlay, options1){};
    ICBSSearchWithPairedAnalysis(ICBSSearchWithPairedAnalysis<Map>* engine,int node_limit);
    ICBSNode* a2_node;
};




