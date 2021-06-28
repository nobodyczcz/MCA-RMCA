// Load's a 2D map.
#pragma once

#include <string>
#include <vector>
#include "map_loader.h"
#include <boost/python.hpp>

namespace p = boost::python;
using namespace std;

struct railCell {
	int transitions;
	bool isTurn;
	bool isDeadEnd;
};

class FlatlandLoader:public MapLoader {
public:
	FlatlandLoader(boost::python::object rail1, int rows, int cols);
	railCell get_full_cell(int location);
	FlatlandLoader();
	int getDegree(int loc);

	boost::python::object rail;
	railCell* railMap;
	vector<pair<int, int>> get_transitions(int location, int heading = -1, bool noWait=false) const;
	~FlatlandLoader();
protected:
	float blockRate;
};

