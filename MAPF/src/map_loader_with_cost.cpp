//
// Created by Zhe Chen on 13/6/20.
//

#include "map_loader_with_cost.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>


void MapLoaderCost::loadKiva(string fname)
{
    string line;
    ifstream myfile (fname.c_str());
    if (!myfile.is_open())
    {
        cerr << "Map file not found." << endl;
        return;
    }
    //read file
    getline(myfile, line);
    boost::char_separator<char> sep(",");
    boost::tokenizer< boost::char_separator<char> > tok(line, sep);
    boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
    rows = atoi((*beg).c_str()) + 2; // read number of rows
    beg++;
    cols = atoi((*beg).c_str()) + 2; // read number of cols

    stringstream ss;
    getline(myfile, line);
    ss << line;
    ss >> workpoint_num;

    int agent_num;
    ss.clear();
    getline(myfile, line);
    ss << line;
    ss >> agent_num;

    ss.clear();
    getline(myfile, line);
    ss << line;
    ss >> maxtime;

    bool* my_map= new bool[rows*cols];
    //DeliverGoal.resize(row*col, false);
    // read map
    int ep = 0, ag = 0;
    for (int i = 1; i<rows - 1; i++)
    {
        getline(myfile, line);
        for (int j = 1; j<cols - 1; j++)
        {
            my_map[cols*i + j] = (line[j - 1] == '@'); // not a block

            if (line[j - 1] == 'e') //endpoint
            {
                endpoints.push_back( i*cols+ j);
                assert(ep == (endpoints.size()-1));
                ep++;

            }
        }

    }
    myfile.close();

    //set the border of the map blocked
    for (int i = 0; i < rows; i++)
    {
        my_map[i*cols] = false;
        my_map[i*cols + cols - 1] = false;
    }
    for (int j = 1; j < cols - 1; j++)
    {
        my_map[j] = false;
        my_map[rows*cols - cols + j] = false;
    }
    this->my_map = my_map;
    // Possible moves [WAIT, NORTH, EAST, SOUTH, WEST]
    moves_offset = new int[MapLoader::MOVE_COUNT];
    moves_offset[MapLoader::valid_moves_t::WAIT_MOVE] = 0;
    moves_offset[MapLoader::valid_moves_t::NORTH] = -cols;
    moves_offset[MapLoader::valid_moves_t::EAST] = 1;
    moves_offset[MapLoader::valid_moves_t::SOUTH] = cols;
    moves_offset[MapLoader::valid_moves_t::WEST] = -1;

    this->initializeKivaMapCost();
}

int MapLoaderCost::getDistance(int initial, int target, int heading) {
    assert(target<cost_map.size());
    if(cost_map[target].empty()){
        setCostMap(target,heading);
    }
    assert(initial < cost_map[target].size());
    return cost_map[target][initial].get_hval(heading);
}

void MapLoaderCost::initializeKivaMapCost(){
    cost_map.resize(rows*cols);
    for (int e : endpoints){
        setCostMap(e);
    }
}

void MapLoaderCost::setCostMap(int location, int heading){
    if(cost_map.empty()){
        cost_map.resize(rows*cols);
    }
    ComputeHeuristic<MapLoader> ch(0, location, this, heading);
    assert(cost_map[location].empty());
    assert(location<cost_map.size());

    ch.getHVals(cost_map[location]);
}



