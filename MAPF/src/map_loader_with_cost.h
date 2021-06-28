//
// Created by Zhe Chen on 13/6/20.
//

#include "map_loader.h"
#include "compute_heuristic.h"


#ifndef MAPD_MAP_LOADER_WITH_COST_H
#define MAPD_MAP_LOADER_WITH_COST_H
class MapLoaderCost:public MapLoader {
public:
    MapLoaderCost(){};
    MapLoaderCost(std::string fname):MapLoader(fname){};
    vector<int> endpoints;
    vector<vector<hvals>> cost_map;
    void loadKiva(string fname);
    void initializeKivaMapCost();
    int getDistance(int initial, int target, int heading = -1);
    void setCostMap(int location, int heading = -1);
};
#endif //MAPD_MAP_LOADER_WITH_COST_H

