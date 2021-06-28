#pragma once
#include "Conflict.h"
#include <climits>
#include <unordered_set>
#include <unordered_map>

class ConstraintTable
{
public:
	int length_min = 0;
	int length_max = INT_MAX;
	int goal_location;
	int latest_timestep = 0; // No negative constraints after this timestep.

	void clear(){
		CT.clear();
		length_min = 0,
		length_max = INT_MAX/2;
		latest_timestep = 0;
		CT_Single.clear();
	}
	void insert(int loc, int t_min, int t_max);
	void insert(std::list<Constraint>& constraints, int agent_id, int num_col, int map_size);
    bool insert_path(int agent_id, const Path& path);
    void delete_path(int agent_id, const Path& path);
	bool is_constrained(int loc, int t);
	bool is_path_constrained(int agent_id, int loc,int pre_loc, int t);
    bool is_dock_safe(int agent_id, int loc, int t);

    void dock(int loc,int agent_id, int time){
	    assert(CT_docking[loc].first == -1);
	    CT_docking[loc].first = agent_id;
        CT_docking[loc].second = time;
	}
	void delete_dock(int loc, int agent_id){
	    assert(agent_id == CT_docking[loc].first || CT_docking[loc].first==-1);
        CT_docking[loc].first = -1;
        CT_docking[loc].second = -1;

	}
	void printSize() {
		std::cout << CT.size() << std::endl;;
	};
    void init(size_t map_size)
    {
        CT_paths.resize(map_size);
        CT_docking.resize(map_size,pair<int,int>(-1,-1));
    }
private:

	std::unordered_map<size_t, std::unordered_set<int> > CT_Single;
	std::unordered_map<size_t, list<pair<int, int> > > CT;
    vector< std::unordered_map<int,int> > CT_paths; // this stores the already planned paths, the value is the id of the agent
    vector< std::pair<int,int>> CT_docking;


};

