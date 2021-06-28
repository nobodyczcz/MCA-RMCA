//
// Created by Zhe Chen on 10/6/20.
//

#ifndef MAPD_CBSHONLINE_H
#define MAPD_CBSHONLINE_H

#include "ICBSSearch.h"
#include "ICBSNode.h"
#include "map_loader.h"
//#include "flat_map_loader.h"
template <typename Map>
class CBSHOnline : public MultiMapICBSSearch<Map> {
public:
    void clear();
    CBSHOnline(Map * ml, AgentsLoader & al, double f_w, constraint_strategy c, int time_limit, int screen,int kDlay, options options1);
    void initializeSearchEngine();
    void initializeDummyStart();
    void set_focal_w(float f_w);

};
#endif //MAPD_CBSHONLINE_H
