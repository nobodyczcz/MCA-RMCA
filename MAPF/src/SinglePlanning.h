#pragma once

#include <stdlib.h>

#include <vector>
#include <list>
#include <utility>
#include <ctime>
#include <chrono>
#include "common.h"
#include "agents_loader.h"
#include "LLNode.h"
#include "ConstraintTable.h"
#include "compute_heuristic.h"
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_set.hpp>
#include "map_loader_with_cost.h"
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;


class SinglePlanning
{
public:
	// define typedefs and handles for heap and hash_map
	typedef boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::secondary_compare_node> > heap_focal_t;
    typedef boost::unordered_set<LLNode*, LLNode::NodeHasher, LLNode::eqnode> hashtable_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	hashtable_t allNodes_table;

	int start_location;
	int goal_location;
	int min_end_time;
	int agent_id;
	int start_time;

    vector<PathEntry> path;

    MapLoaderCost& ml;
	int map_size;
	int num_col;
    ConstraintTable& constraintTable;

    int screen;
    float time_limit;
	float f_w;

	uint64_t LL_num_expanded;
	uint64_t LL_num_generated;

	double focal_threshold;  // FOCAL's lower bound ( = e_weight * min_f_val)
	double min_f_val;  // min f-val seen so far
	int num_of_conf; // number of conflicts between this agent to all the other agents


	//Checks if a vaild path found (wrt my_map and constraints)
	//Note -- constraint[timestep] is a list of pairs. Each pair is a disallowed <loc1,loc2> (loc2=-1 for vertex constraint).
	//bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >* cons) const;
	inline bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >* cons)  const
	{
		if (cons == nullptr)
			return false;
		// check vertex constraints (being in next_id at next_timestep is disallowed)
		if (next_timestep < static_cast<int>(cons->size()))
		{
			for (const auto & it : cons->at(next_timestep))
			{
				if ((std::get<0>(it) == next_id && std::get<1>(it) < 0)//vertex constraint
					|| (std::get<0>(it) == curr_id && std::get<1>(it) == next_id)) // edge constraint
					return true;
			}
		}
		return false;
	}

	// Updates the path datamember
	void updatePath(LLNode* goal);

	// find path by time-space A* search
	// Returns true if a collision free path found  while
	// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
	// lowerbound is the lowerbound of the length of the path
	// max_plan_len used to compute the size of res_table
	bool search(bool docking = false);

	inline void releaseClosedListNodes(hashtable_t* allNodes_table);


    SinglePlanning(MapLoaderCost& ml,int agent_id, int start, int goal, int start_time, int min_end_time, double f_w, float time_limit, options option, ConstraintTable& constraintTable);


};


