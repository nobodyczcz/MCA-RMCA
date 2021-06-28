#pragma once

#include <vector>
#include <utility>
#include <stdlib.h>
#include "map_loader.h"
//#include "flat_map_loader.h"

#include <boost/unordered_map.hpp>


using namespace std;


template<class Map>
class ComputeHeuristic 
{
 public:
  int start_location;
  int goal_location;
  int start_heading;
  Map* ml;
  int map_rows;
  int map_cols;
  ComputeHeuristic();
  ComputeHeuristic(int start_location, int goal_location, Map* ml0, int start_heading = 4);
 
 bool validMove(int curr, int next) const;

 void getHVals(vector<hvals>& res,int limit = INT_MAX);


  ~ComputeHeuristic();

};


