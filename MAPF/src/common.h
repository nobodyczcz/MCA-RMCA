#pragma once

#include <tuple>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_map.hpp>
#include <google/dense_hash_map>
#include <iostream>
#include <unordered_map>
#include <list>
#include <vector>
#include <unordered_set>
enum conflict_type { TARGET, CORRIDOR2, CORRIDOR4, RECTANGLE,RECTANGLE4, STANDARD, TYPE_COUNT };
enum conflict_priority { CARDINAL, SEMI, NON, UNKNOWN, PRIORITY_COUNT };

enum constraint_type { LENGTH, RANGE, BARRIER, VERTEX, EDGE, CONSTRAINT_COUNT };
typedef std::tuple<int, int, int, constraint_type> Constraint;
typedef std::list<std::shared_ptr<std::tuple<int, int, int, int, int,int>>> OldConfList;

enum constraint_strategy { CBS, ICBS, CBSH, CBSH_CR, CBSH_R, CBSH_RM, CBSH_GR,PBS,PP,REGRET, STRATEGY_COUNT };
enum action { UP, RIGHT, DOWN, LEFT, WAIT };


using std::pair;
using std::make_pair;
using std::unordered_map;
using std::list;
using std::vector;
using std::tuple;

namespace N
{
	template<typename T>
	void get(T); //no need to provide definition
				 // as far as enabling ADL is concerned!
}
struct options {
    bool asymmetry_constraint=false;
    bool debug = false;
    bool ignore_t0 = false;
    bool shortBarrier = false;
    bool flippedRec = false;
    bool multi_label = false;
    int RM4way;
    bool pairAnalysis = false;
    bool printFailedPair = false;
    bool ignore_target = false;
    int window_size = 0;
    float f_w = 1;
    constraint_strategy s;
};


struct PathEntry
{

	int location;
	int heading;
	bool single;
	int actionToHere;
	int timeStep=0;
	PathEntry(int loc = -1) { location = loc; single = false; }
	std::list<int> locations; // all possible locations at the same time step
	OldConfList* conflist=NULL;
};

struct hvals {
    boost::unordered_map<int, int> heading;
    int get_hval(int direction) {
        if (heading.count(direction)) {
            return heading[direction];
        }
        else {
            return INT_MAX/2;
        }
    }
    ~hvals(){
        heading.clear();
    }
};

struct MDDPath {
	vector<std::unordered_set<int>> levels;
	void print() {
		for (int l = 0; l < levels.size(); l++) {
			std::unordered_set<int>::iterator it;
			std::cout << "level " << l << ": ";
			for (it = levels[l].begin(); it != levels[l].end(); ++it) {
				std::cout << *it << ",";
			}
			std::cout << std::endl;
		}
	}
	void print(int col) {
		for (int l = 0; l < levels.size(); l++) {
			std::unordered_set<int>::iterator it;
			std::cout << "level " << l << ": ";
			for (it = levels[l].begin(); it != levels[l].end(); ++it) {
				std::cout <<"("<< *it/col << "," << *it%col <<") ";
			}
			std::cout << std::endl;
		}
	}
};



typedef std::vector<PathEntry> Path;


 //Only for three-tuples of std::hash-able types for simplicity.
 //You can of course template this struct to allow other hash functions
struct three_tuple_hash {
	template <class T1, class T2, class T3>
	std::size_t operator () (const std::tuple<T1, T2, T3> &p) const {
		using N::get;

		auto h1 = std::hash<T1>{}(get<0>(p));
		auto h2 = std::hash<T2>{}(get<1>(p));
		auto h3 = std::hash<T3>{}(get<2>(p));
		// Mainly for demonstration purposes, i.e. works but is overly simple
		// In the real world, use sth. like boost.hash_combine
		return h1 ^ h2 ^ h3;
	}
};

int getLocation(const std::vector<PathEntry>& path, int timestep);

int getMahattanDistance(int loc1_x, int loc1_y, int loc2_x, int loc2_y);
int getArea(int loc1_x, int loc1_y, int loc2_x, int loc2_y);

int getAction(int loc, int pre_loc, int num_col);