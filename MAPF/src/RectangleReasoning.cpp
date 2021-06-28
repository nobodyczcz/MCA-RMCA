#include "RectangleReasoning.h"
#include <algorithm>    // std::find
#include <iostream>
#include <ctime> //std::time
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_set.hpp>



/*
 * This part check the relationship between MDD and barriers.
 */

//Return true if a barrier fully cut MDD
bool isCut(MDDLevels& mdd, std::list<Constraint>& constraints,int num_col, int map_size){
    ConstraintTable consTable;
    consTable.insert(constraints, 0, num_col, map_size);
    list<MDDNode*> queue;
    boost::unordered_set<MDDNode*> explored;

    queue.push_front(mdd[0].front());
    MDDNode* goal = mdd.back().back();

    while(!queue.empty()){
        MDDNode* current = queue.front();
        explored.insert(queue.front());
        queue.pop_front();

        for (auto child : current->children){
            if (child == goal){
                    return false;
            }
            if (explored.count(child)==0 && !consTable.is_constrained(child->location,child->level)){
                queue.push_front(child);
            }
        }

    }
    return true;

}

/// find solution with entrance constrained;
/// \param mdd
/// \param entrance
/// \param entranceTable
/// \param exit
/// \param exitTable
/// \return -1 if have solution traverse exit, 0 if only solution bypass exit, 1 if no any solution;
int haveSolutionCondition1(const MDDLevels& mdd,
        std::list<Constraint>& entrance,
        ConstraintTable& entranceTable,
        std::list<Constraint>& exit,
        ConstraintTable& exitTable)
{


    list<MDDNode*> queue;
    boost::unordered_set<MDDNode*> explored;
    boost::unordered_set<MDDNode*> traverse_exit;

    MDDNode* goal = mdd.back().back();



    queue.push_front(mdd[0].front());
    explored.insert(mdd[0].front());

    if(entranceTable.is_constrained(queue.front()->location,queue.front()->level)){
        return 1;
    }

    bool solutionTraverseExit = false;
    bool solutionBypassExit = false;
    while(!queue.empty()){
        MDDNode* current = queue.front();
        queue.pop_front();

        for (auto child : current->children){
            if (child == goal){
                if(exitTable.is_constrained(child->location,child->level) || traverse_exit.count(current) > 0)
                    solutionTraverseExit = true;
                else
                    solutionBypassExit = true;
            }
            if (explored.count(child)==0 && !entranceTable.is_constrained(child->location,child->level)){

                if(exitTable.is_constrained(child->location,child->level) || traverse_exit.count(current) > 0){
                    traverse_exit.insert(child);
                    queue.push_front(child);
                    explored.insert(child);

                }
                else {
                    queue.push_front(child);
                    explored.insert(child);

                }
            }
        }

    }
    if (solutionTraverseExit)
        return -1;
    else if (solutionBypassExit)
        return 0;
    else
        return 1;
};

//Return true if find solution with exit constrained and must traverse entrance;
bool haveSolutionCondition2(const MDDLevels& mdd,
                              std::list<Constraint>& entrance,
                              ConstraintTable& entranceTable,
                              std::list<Constraint>& exit,
                              ConstraintTable& exitTable
                              )
{

    list<MDDNode*> queue;
    boost::unordered_set<MDDNode*> explored;
    boost::unordered_set<MDDNode*> traverse_entrance;

    MDDNode* goal = mdd.back().back();

    int max_entrance_time = 0;
    for (auto constraint : entrance){
        if (get<2>(constraint) > max_entrance_time)
            max_entrance_time = get<2>(constraint);
    }

    queue.push_front(mdd[0].front());
    explored.insert(mdd[0].front());

    if(entranceTable.is_constrained(queue.front()->location,queue.front()->level)){
        traverse_entrance.insert(queue.front());
    }
    while(!queue.empty()){
        MDDNode* current = queue.front();
        queue.pop_front();

        for (auto child : current->children){
            if (child == goal){
                if(entranceTable.is_constrained(child->location,child->level) || traverse_entrance.count(current) > 0)
                    return true;
            }
            if (explored.count(child)==0 && !exitTable.is_constrained(child->location,child->level)){

                if(entranceTable.is_constrained(child->location,child->level) || traverse_entrance.count(current) > 0){
                    traverse_entrance.insert(child);
                    queue.push_front(child);
                    explored.insert(child);

                }
                else if(child->level < max_entrance_time){
                    queue.push_front(child);
                    explored.insert(child);
                }
            }
        }

    }
    return false;
};

int classifyBarrierAndRectangle(const MDDLevels& mdd, std::list<Constraint>& entrance, std::list<Constraint>& exit, int num_col, int map_size){
    ConstraintTable entranceTable;
    entranceTable.insert(entrance, 0, num_col, map_size);

    ConstraintTable exitTable;
    exitTable.insert(exit, 0, num_col, map_size);
    int condition1Result = haveSolutionCondition1(mdd, entrance, entranceTable, exit, exitTable);
    if (condition1Result == -1){
        return -1;
    }
    else{
        if (haveSolutionCondition2(mdd, entrance, entranceTable, exit, exitTable) || condition1Result == 0)
            return 0;
        else
            return 1;
    }


}





//Identify rectangle conflicts for CR/R
bool isRectangleConflict(const std::pair<int, int>& s1, const std::pair<int, int>& s2,
	const std::pair<int, int>& g1, const std::pair<int, int>& g2, int g1_t, int g2_t)
{
	return g1_t == abs(s1.first - g1.first) + abs(s1.second - g1.second) &&  // Manhattan-optimal
		        g2_t == abs(s2.first - g2.first) + abs(s2.second - g2.second) && // Manhattan-optimal
			(s1.first - g1.first) * (s2.first - g2.first) >= 0 &&  //Move in the same direction
			(s1.second - g1.second) * (s2.second - g2.second) >= 0; //Move in the same direction
}

//Identify rectangle conflicts for RM
bool isRectangleConflict(int s1, int s2, int g1, int g2, int num_col,int kRobust,int tDifference,bool I_RM)
{
	if (s1 == s2) // A standard cardinal conflict
		return false;
	else if (s1 == g1 || s2 == g2) // s1 = g1 or  s2 = g2
		return false;
	int s1_x = s1 / num_col, s1_y = s1 % num_col;
	int s2_x = s2 / num_col, s2_y = s2 % num_col;
	int g1_x = g1 / num_col, g1_y = g1 % num_col;
	int g2_x = g2 / num_col, g2_y = g2 % num_col;
	//cout << "(" << s1_x << "," << s1_y << ")" << "(" << g1_x << "," << g1_y << ")" << endl;
	//cout << "(" << s2_x << "," << s2_y << ")" << "(" << g2_x << "," << g2_y << ")" << endl;
	if ((s1_x - g1_x) * (s2_x - g2_x) < 0 || (s1_y - g1_y) * (s2_y - g2_y) < 0) { // Not move in the same direction
		return false;
	}
	else if ((s2_x - s1_x) * (s1_x - g1_x) < 0 && (s2_y - s1_y) * (s1_y - g1_y) < 0) { // s1 always in the middle, s2 always between s1 and g1
		if (!I_RM)
			return false;

        return !(((s2_x - g1_x) * (g1_x - g2_x) < 0 && (s2_y - g1_y) * (g1_y - g2_y) < 0)
                 || ((s2_x - g2_x) * (g2_x - g1_x) < 0 && (s2_y - g2_y) * (g2_y - g1_y) < 0));


    }
	else if ((s1_x - s2_x) * (s2_x - g2_x) < 0 && (s1_y - s2_y) * (s2_y - g2_y) < 0) { // s2 always in the middle, s1 always between s2 and g2
		if (!I_RM)
			return false;

        return !(((s1_x - g1_x) * (g1_x - g2_x) < 0 && (s1_y - g1_y) * (g1_y - g2_y) < 0)
                 || ((s1_x - g2_x) * (g2_x - g1_x) < 0 && (s1_y - g2_y) * (g2_y - g1_y) < 0));


    }
	else if ((s1_x == g1_x && s2_y == g2_y) || (s1_y == g1_y && s2_x == g2_x)) { // area = 1

		return false;
	}
	else {
		return true;
	}
}

/*
 * Retrieve st and gt for new rm
 */

pair<int,int> get_st(const std::vector<PathEntry>& path, int timestep, int num_col, int action1, int action2, int kDelay, bool single_only, bool pause_on_stop) {
	pair<int, int> result;
	int candidate=-1;
	result.second == 0;
	int preAction = -1;
	int wait_count = kDelay;
	for (int t = timestep; t > 0; t--) {
		int action = getAction(path[t].location, path[t - 1].location, num_col);
		if (action != action1 && action != action2 && (action!=action::WAIT || wait_count == 0 || pause_on_stop)) {
            result.first = candidate;
            return result;
        }
        if(action == action::WAIT && wait_count > 0 && kDelay > 0)
		    wait_count --;

		
		if (!single_only || path[t].single){
		    candidate = t;
		}
		if (preAction != -1 && action != preAction) {
			result.second++;
		}
		preAction = action;
	}

	//st is start location
	result.first = 0;
	//result.second++;
	return result;
	
};
pair<int, int> get_gt(const std::vector<PathEntry>& path, int timestep, int num_col, int action1, int action2, int kDelay, bool single_only,bool pause_on_stop) {
	pair<int, int> result;
    int candidate=-1;
    result.second == 0;
	int preAction = -1;
	int wait_count = kDelay;
	for (int t = timestep; t < path.size()-1; t++) {
		int action = getAction(path[t + 1].location, path[t].location, num_col);
		if (action != action1 && action != action2 && (action!=action::WAIT || wait_count == 0|| pause_on_stop)) {
			result.first = candidate;
			return result;
		}

        if(action == action::WAIT && wait_count > 0 && kDelay > 0)
            wait_count --;
        if (!single_only || path[t].single){
            candidate = t;
        }
		if (preAction != -1 && action != preAction) {
			result.second++;
		}
		preAction = action;
	}
	result.first = path.size() - 1;
	//result.second++;
	return result;

};

MDDNode* get_st_mdd(const MDDLevels& mdd, int timestep,int loc, int num_col, int action1, int action2){
    MDDNode* start;
    for (auto node : mdd[timestep]){
        if (node->location == loc)
            start = node;
    }

    list<MDDNode*> queue;
    boost::unordered_set<MDDNode*> explored;

    queue.push_back(start);
    explored.insert(start);
    MDDNode* furthest_node = start;
    int distance = 0;

    while (!queue.empty()){
        MDDNode* current = queue.front();
        queue.pop_front();
        int current_distance = getMahattanDistance(current->row,current->col,loc/num_col,loc%num_col);
        if(current_distance > distance){
            furthest_node = current;
            distance = current_distance;
        }
        for (auto node : current->parents){
            int action = getAction(current->location, node->location, num_col);
            if ((action == action1 || action == action2) && explored.count(node) == 0) {
                queue.push_back(node);
                explored.insert(node);

            }
        }

    }
    return furthest_node;

};
MDDNode* get_gt_mdd(const MDDLevels& mdd, int timestep,int loc, int num_col, int action1, int action2){
    MDDNode* start;
    for (auto node : mdd[timestep]){
        if (node->location == loc)
            start = node;
    }

    list<MDDNode*> queue;
    boost::unordered_set<MDDNode*> explored;
    queue.push_back(start);
    explored.insert(start);

    MDDNode* furthest_node = start;
    int distance = 0;

    while (!queue.empty()){
        MDDNode* current = queue.front();
        queue.pop_front();

        int current_distance = getMahattanDistance(current->row,current->col,loc/num_col,loc%num_col);
        if(current_distance > distance){
            furthest_node = current;
            distance = current_distance;
        }
        for (auto node : current->children){
            int action = getAction( node->location,current->location, num_col);
            if ((action == action1 || action == action2) && explored.count(node) == 0) {
                queue.push_back(node);
                explored.insert(node);

            }
        }

    }
    return furthest_node;
};

std::pair<MDDNode*,MDDNode*> get_sg_mdd(const MDDLevels& mdd, int timestep,int loc, int num_col, int action1, int action2) {
    MDDNode* start;
    for (auto node : mdd[timestep]){
        if (node->location == loc)
            start = node;
    }

    list<MDDNode*> st_queue;
    list<MDDNode*> gt_queue;

    boost::unordered_set<MDDNode*> explored;
    st_queue.push_back(start);
    gt_queue.push_back(start);
    explored.insert(start);

    list<MDDNode*> candidate_st;
    list<MDDNode*> candidate_gt;

    while (!st_queue.empty() or !gt_queue.empty()){
        if (!st_queue.empty()){
            MDDNode* current = st_queue.front();
            st_queue.pop_front();

            bool no_valid = true;
            for (auto node : current->parents){
                int action = getAction(current->location, node->location, num_col);
                if (action == action1 || action == action2){
                    no_valid = false;
                    if (explored.count(node) == 0) {
                        st_queue.push_back(node);
                        explored.insert(node);
                    }
                }

            }
            if (no_valid)
                candidate_st.push_back(current);
        }

        if(!gt_queue.empty()){
            MDDNode* current = gt_queue.front();
            gt_queue.pop_front();

            bool no_valid = true;
            for (auto node : current->children){
                int action = getAction( node->location,current->location, num_col);
                if (action == action1 || action == action2){
                    no_valid = false;
                    if (explored.count(node) == 0) {
                        gt_queue.push_back(node);
                        explored.insert(node);
                    }
                }

            }
            if (no_valid)
                candidate_gt.push_back(current);
        }
    }

    int area = -1;
    MDDNode* best_st;
    MDDNode* best_gt;
    for (MDDNode* st:candidate_st){
        for(MDDNode* gt:candidate_gt){
            int new_area = getArea(st->row,st->col,gt->row,gt->col);
            if (new_area > area){
                area = new_area;
                best_st = st;
                best_gt = gt;
            }
        }
    }

    return std::make_pair(best_st,best_gt);
}


int get_earlyCrosst(const std::vector<PathEntry>& path1, const std::vector<PathEntry>& path2, int timestep, int earlyBound, int delta) {
	for (int t = timestep-1; t > earlyBound; t--) {
		if (path1[t].location == path2[t+delta].location) {
			return t;
		}
	}
	return -1;
};
int get_lateCrosst(const std::vector<PathEntry>& path1, const std::vector<PathEntry>& path2, int timestep, int lateBound, int delta) {
	for (int t = timestep+1; t < lateBound; t++) {
		if (path1[t].location == path2[t + delta].location) {
			return t;
		}
	}
	return -1;
};



//Classify rectangle conflicts for CR/R
// Return 2 if it is a cardinal rectangle conflict
// Return 1 if it is a semi-cardinal rectangle conflict
// Return 0 if it is a non-cardinal rectangle conflict
int classifyRectangleConflict(const std::pair<int, int>& s1, const std::pair<int, int>& s2,
	const std::pair<int, int>& g1, const std::pair<int, int>& g2)
{
	int cardinal1 = 0, cardinal2 = 0;
	if ((s1.first - s2.first) * (g1.first - g2.first) <= 0)
		cardinal1++;
	if ((s1.second - s2.second) * (g1.second - g2.second) <= 0)
		cardinal2++;
	return cardinal1 + cardinal2;
}

//Classify rectangle conflicts for RM
// Return 2 if it is a cardinal rectangle conflict
// Return 1 if it is a semi-cardinal rectangle conflict
// Return 0 if it is a non-cardinal rectangle conflict
int classifyRectangleConflict(int s1, int s2, int g1, int g2, const std::pair<int, int>& Rg, int num_col,bool I_RM)
{
	int cardinal1 = 0, cardinal2 = 0;

	int s1_x = s1 / num_col, s1_y = s1 % num_col;
	int s2_x = s2 / num_col, s2_y = s2 % num_col;
	int g1_x = g1 / num_col, g1_y = g1 % num_col;
	int g2_x = g2 / num_col, g2_y = g2 % num_col;

	if ((s2_x - s1_x) * (s1_x - g1_x) < 0 && (s2_y - s1_y) * (s1_y - g1_y) < 0) { // s1 always in the middle
		cardinal1 = 1;
		cardinal2 = 0;
	}
	else if ((s1_x - s2_x) * (s2_x - g2_x) < 0 && (s1_y - s2_y) * (s2_y - g2_y) < 0) { // s2 always in the middle
		cardinal1 = 1;
		cardinal2 = 0;
	}
	else if ((s1_x == s2_x && (s1_y - s2_y) * (s2_y - Rg.second) >= 0) ||
		(s1_x != s2_x && (s1_x - s2_x)*(s2_x - Rg.first) < 0))
	{
		if (Rg.first == g1_x)
			cardinal1 = 1;
		if (Rg.second == g2_y)
			cardinal2 = 1;
	}
	else
	{
		if (Rg.second == g1_y)
			cardinal1 = 1;
		if (Rg.first == g2_x)
			cardinal2 = 1;
	}

	return cardinal1 + cardinal2;
}

//Compute rectangle corner Rs
std::pair<int, int> getRs(const std::pair<int, int>& s1, const std::pair<int, int>& s2, const std::pair<int, int>& g1)
{
	int x, y;
	if (s1.first == g1.first)
		x = s1.first;
	else if (s1.first < g1.first)
		x = std::max(s1.first, s2.first);
	else
		x = std::min(s1.first, s2.first);
	if (s1.second == g1.second)
		y = s1.second;
	else if (s1.second < g1.second)
		y = std::max(s1.second, s2.second);
	else
		y = std::min(s1.second, s2.second);
	return std::make_pair(x, y);
}

//Compute rectangle corner Rg
std::pair<int, int> getRg(const std::pair<int, int>& s1, const std::pair<int, int>& g1, const std::pair<int, int>& g2)
{
	int x, y;
	if (s1.first == g1.first)
		x = g1.first;
	else if (s1.first < g1.first)
		x = std::min(g1.first, g2.first);
	else
		x = std::max(g1.first, g2.first);
	if (s1.second == g1.second)
		y = g1.second;
	else if (s1.second < g1.second)
		y = std::min(g1.second, g2.second);
	else
		y = std::max(g1.second, g2.second);
	return std::make_pair(x, y);
}

//Identify flipped rectangle conflicts for RM
int isFlippedRectangleConflict(int s1, int s2, int g1, int g2, int num_col)
{
	if (s1 == s2) // A standard cardinal conflict
		return -4;
	else if (s1 == g1 || s2 == g2) // s1 = g1 or  s2 = g2
		return -4;
	int s1_x = s1 / num_col, s1_y = s1 % num_col;
	int s2_x = s2 / num_col, s2_y = s2 % num_col;
	int g1_x = g1 / num_col, g1_y = g1 % num_col;
	int g2_x = g2 / num_col, g2_y = g2 % num_col;
	//cout << "(" << s1_x << "," << s1_y << ")" << "(" << g1_x << "," << g1_y << ")" << endl;
	//cout << "(" << s2_x << "," << s2_y << ")" << "(" << g2_x << "," << g2_y << ")" << endl;

	
	if ((s1_x == g1_x && s2_y == g2_y) || (s1_y == g1_y && s2_x == g2_x)) { // area = 1

		return -1;
	}
	else if ((s1_x - g1_x) * (s2_x - g2_x) < 0 && (s1_y - g1_y) * (s2_y - g2_y) < 0) { // 2 flipped
		/*if (
			(s2_x - g1_x) * (g1_x - g2_x) < 0 
			&& (s2_x - s1_x) * (s1_x - g2_x) < 0
			&& (g1_x - s2_x) * (s1_x - s2_x)>0
			) {

			return -3;
		}
		else if ((s2_y - g1_y) * (g1_y - g2_y) < 0
			&& (s2_y - s1_y) * (s1_y - g2_y) < 0
			&& (g1_y - s2_y) * (s1_y - s2_y) > 0
			) { 
			return -3;
		}*/
		if ((g2_x - s1_x) * (s1_x - g1_x) < 0 && (g2_y - s1_y) * (s1_y - g1_y) < 0) { // s1 always in the middle


			return -3;
		}
		else if ((g1_x - s2_x) * (s2_x - g2_x) < 0 && (g1_y - s2_y) * (s2_y - g2_y) < 0) { // s2 always in the middle

			return -3;
		}
		return 2;
	}
	else if (((s1_x - g1_x) * (s2_x - g2_x) < 0 || (s1_y - g1_y) * (s2_y - g2_y) < 0) && ((s1_x - g1_x) * (s2_x - g2_x) != 0 && (s1_y - g1_y) * (s2_y - g2_y) != 0)) { // 1 flipped

		if ((s1_y - g1_y) * (s2_y - g2_y) < 0) {
			if ((s1_y - s2_y)*(s2_y - g2_y) > 0 || (s2_x - g2_x)*(g2_x - s1_x) > 0)
			return -2;
		}
		else {
			if ((s1_x - s2_x)*(s2_x - g2_x) > 0 || (s2_y - g2_y)*(g2_y - s1_y) > 0)
				return -2;
		}
		return 1;
	}
	else {
		/*if (
			(s2_x - g1_x) * (g1_x - g2_x) < 0
			&& (s2_x - s1_x) * (s1_x - g2_x) < 0
			&& (g1_x - s2_x) * (s1_x - s2_x) > 0
			) {

			return -1;
		}
		else if ((s2_y - g1_y) * (g1_y - g2_y) < 0
			&& (s2_y - s1_y) * (s1_y - g2_y) < 0
			&& (g1_y - s2_y) * (s1_y - s2_y) > 0
			) {
			return -1;
		}*/
		if ((s2_x - s1_x) * (s1_x - g1_x) < 0 && (s2_y - s1_y) * (s1_y - g1_y) < 0) { // s1 always in the middle

			return -1;
		}
		else if ((s1_x - s2_x) * (s2_x - g2_x) < 0 && (s1_y - s2_y) * (s2_y - g2_y) < 0) { // s2 always in the middle

			return -1;
		}
		return 0;
	}
		
}

//Classify rectangle conflicts for flipped RM
// Return 2 if it is a cardinal rectangle conflict
// Return 1 if it is a semi-cardinal rectangle conflict
// Return 0 if it is a non-cardinal rectangle conflict
int classifyFlippedRectangleConflict(int s1, int s2, int g1, int g2, const std::pair<int, int>& Rg, const std::pair<int, int>& Rs, int num_col, int flipType, bool kFullyBlocked)
{
	if (!kFullyBlocked)
		return 0;
	int cardinal1 = 0, cardinal2 = 0;

	int s1_x = s1 / num_col, s1_y = s1 % num_col;
	int s2_x = s2 / num_col, s2_y = s2 % num_col;
	int g1_x = g1 / num_col, g1_y = g1 % num_col;
	int g2_x = g2 / num_col, g2_y = g2 % num_col;

	if (flipType == 2) {
		if (s1_x == s2_x || s1_y == s2_y)// for two flipped case, both g1 and g2 can not in same line with s1 s2 (it will be a one flipped case).
			return 0;
		
		if (Rs.first == s1_x && Rs.second == s1_y) {
			cardinal1 = 1;
		}
		else if (Rs.first == s1_x && Rg.first == g1_x) {
			cardinal1 = 1;
		}
		else if (Rs.second == s1_y && Rg.second == g1_y) {
			cardinal1 = 1;
		}
		else if (Rg.first == g1_x && Rg.second == g1_y) {
			cardinal1 = 1;
		}

		if (Rg.first == s2_x && Rg.second == s2_y) {
			cardinal2 = 1;
		}
		else if (Rg.first == s2_x && Rs.first==g2_x){
			cardinal2 = 1;
		}
		else if(Rg.second == s2_y && Rs.second==g2_y){
			cardinal2 = 1;
		}
		else if (Rs.first == g2_x && Rs.second == g2_y) {
			cardinal2 = 1;
		}
			

	}
	else if (flipType == 1) {
		if ((s1_y - g1_y) * (s2_y - g2_y) < 0) {//y dimension flipped
			if (Rs.first == s1_x) {
				if (Rg.second == s1_y)
					cardinal1 = 1;
				else if(Rg.first == g1_x)
					cardinal1 = 1;
			}
			else {
				if (Rg.second == s1_y) {
					if (Rs.second == g1_y)
						cardinal1 = 1;
				}
				else {
					if (Rg.first == g1_x && Rs.second==g1_y)
						cardinal1 = 1;
				}
			}

			if (Rs.first == s2_x) {
				if (Rs.second == s2_y)
					cardinal2 = 1;
				else if (Rg.first == g2_x)
					cardinal2 = 1;
			}
			else {
				if (Rs.second == s2_y) {
					if (Rg.second == g2_y)
						cardinal2 = 1;
				}
				else {
					if (Rg.first == g1_x && Rg.second==g1_y)
						cardinal2 = 1;
				}
			}

		}
		else {//x dimension flipped
			if (Rs.second == s1_y) {
				if (Rg.first == s1_x)
					cardinal1 = 1;
				else if (Rg.second == g1_y)
					cardinal1 = 1;
			}
			else {
				if (Rg.first == s1_x) {
					if (Rs.first == g1_x)
						cardinal1 = 1;
				}
				else {
					if (Rg.second == g1_y && Rs.first == g1_x)
						cardinal1 = 1;
				}
			}

			if (Rs.second == s2_y) {
				if (Rs.first == s2_x)
					cardinal2 = 1;
				else if (Rg.second == g2_y)
					cardinal2 = 1;
			}
			else {
				if (Rs.first == s2_x) {
					if (Rg.first == g2_x)
						cardinal2 = 1;
				}
				else {
					if (Rg.second == g1_y && Rg.first == g1_x)
						cardinal2 = 1;
				}
			}
		}
	}
	else {//non flip
		if ((s1_x == s2_x && (s1_y - s2_y) * (s2_y - Rg.second) >= 0) ||
			(s1_x != s2_x && (s1_x - s2_x)*(s2_x - Rg.first) < 0))
		{
			if (Rg.first == g1_x)
				cardinal1 = 1;
			if (Rg.second == g2_y)
				cardinal2 = 1;
		}
		else
		{
			if (Rg.second == g1_y)
				cardinal1 = 1;
			if (Rg.first == g2_x)
				cardinal2 = 1;
		}

		
	}
	return cardinal1 + cardinal2;

	
}

std::pair<int, int> getFlippedRs(const std::pair<int, int>& s1, const std::pair<int, int>& s2, const std::pair<int, int>& g1, const std::pair<int, int>& g2, int flipType)
{
	int x, y;

	if (flipType == 2) {
		if (s1.first == g1.first)
			x = g1.first;
		else if (s1.first < g1.first)
			x = std::max(s1.first, g2.first);
		else
			x = std::min(s1.first, g2.first);

		if (s1.second == g1.second)
			y = g1.second;
		else if (s1.second < g1.second)
			y = std::max(s1.second, g2.second);
		else
			y = std::min(s1.second, g2.second);
	}
	else if (flipType == 1) {
		if ((s1.second - g1.second) * (s2.second - g2.second) < 0) {//y dimension flipped
			if (s1.first == g1.first)
				x = g1.first;
			else if (s1.first < g1.first)
				x = std::max(s2.first, s1.first);
			else
				x = std::min(s2.first, s1.first);

			if (s1.second == g1.second)
				y = g1.second;
			else if (s1.second < g1.second)
				y = std::min(g1.second, s2.second);
			else
				y = std::max(g1.second, s2.second);
		}
		else {//x dimension flipped
			if (s1.first == g1.first)
				x = g1.first;
			else if (s1.first < g1.first)
				x = std::min(s2.first, g1.first);
			else
				x = std::max(s2.first, g1.first);

			if (s1.second == g1.second)
				y = g1.second;
			else if (s1.second < g1.second)
				y = std::max(s1.second, s2.second);
			else
				y = std::min(s1.second, s2.second);
		}
	}
	else {
		if (s1.first == g1.first)
			x = s1.first;
		else if (s1.first < g1.first)
			x = std::max(s1.first, s2.first);
		else
			x = std::min(s1.first, s2.first);
		if (s1.second == g1.second)
			y = s1.second;
		else if (s1.second < g1.second)
			y = std::max(s1.second, s2.second);
		else
			y = std::min(s1.second, s2.second);
	}

	return std::make_pair(x, y);

}

std::pair<int, int> getFlippedRg(const std::pair<int, int>& s1, const std::pair<int, int>& s2, const std::pair<int, int>& g1, const std::pair<int, int>& g2, int flipType)
{
	int x, y;

	if (flipType==2){
		if (s1.first == g1.first)
			x = g1.first;
		else if (s1.first < g1.first)
			x = std::min(s2.first, g1.first);
		else
			x = std::max(s2.first, g1.first);

		if (s1.second == g1.second)
			y = g1.second;
		else if (s1.second < g1.second)
			y = std::min(g1.second, s2.second);
		else
			y = std::max(g1.second, s2.second);
	}
	else if (flipType == 1) {
		if ((s1.second - g1.second) * (s2.second - g2.second) < 0) {//y dimension flipped
			if (s1.first == g1.first)
				x = g1.first;
			else if (s1.first < g1.first)
				x = std::min(g2.first, g1.first);
			else
				x = std::max(g2.first, g1.first);

			if (s1.second == g1.second)
				y = g1.second;
			else if (s1.second < g1.second)
				y = std::max(g2.second, s1.second);
			else
				y = std::min(g2.second, s1.second);
		}
		else {// x dimension flipped
			if (s1.first == g1.first)
				x = g1.first;
			else if (s1.first < g1.first)
				x = std::max(s1.first, g2.first);
			else
				x = std::min(s1.first, g2.first);

			if (s1.second == g1.second)
				y = g1.second;
			else if (s1.second < g1.second)
				y = std::min(g1.second, g2.second);
			else
				y = std::max(g1.second, g2.second);
		}
		

		
	}
	else {
		if (s1.first == g1.first)
			x = g1.first;
		else if (s1.first < g1.first)
			x = std::min(g1.first, g2.first);
		else
			x = std::max(g1.first, g2.first);

		if (s1.second == g1.second)
			y = g1.second;
		else if (s1.second < g1.second)
			y = std::min(g1.second, g2.second);
		else
			y = std::max(g1.second, g2.second);
	}

	return std::make_pair(x, y);

}

bool isKFullyBlocked(const std::pair<int, int>& s1, const std::pair<int, int>& s2, 
	const std::pair<int, int>& Rs, const std::pair<int, int>& Rg, int k,int s1_t, int s2_t) {
	int c1 = getMahattanDistance(s1.first, s1.second, Rs.first, Rs.second)+s1_t
		- getMahattanDistance(s2.first, s2.second, Rs.first, Rs.second)-s2_t;
	int c2 = getMahattanDistance(s1.first, s1.second, Rs.first, Rg.second) + s1_t
		- getMahattanDistance(s2.first, s2.second, Rs.first, Rg.second) - s2_t;
	int c3 = getMahattanDistance(s1.first, s1.second, Rg.first, Rs.second) + s1_t
		- getMahattanDistance(s2.first, s2.second, Rg.first, Rs.second) - s2_t;
	int c4 = getMahattanDistance(s1.first, s1.second, Rg.first, Rg.second) + s1_t
		- getMahattanDistance(s2.first, s2.second, Rg.first, Rg.second) - s2_t;
	int maxC = max({ abs(c1),abs(c2),abs(c3),abs(c4) });
	if (maxC <= k)
		return true;
	else
		return false;
	

}

//Compute start candidates for RM
std::list<int>  getStartCandidates(const std::vector<PathEntry>& path, int timestep, int num_col)
{
	std::list<int> starts;
	for (int t = 0; t <= timestep; t++) //Find start that is single and Manhattan-optimal to conflicting location
	{
		if (path[t].single && isManhattanOptimal(path[t].location, path[timestep].location, timestep - t, num_col))
			starts.push_back(t);
	}
	return starts;
}

//Compute goal candidates for RM
std::list<int>  getGoalCandidates(const std::vector<PathEntry>& path, int timestep, int num_col)
{
	std::list<int> goals;
	for (int t = path.size() - 1; t >= timestep; t--) //Find start that is single and Manhattan-optimal to conflicting location
	{
		if (path[t].single && isManhattanOptimal(path[t].location, path[timestep].location, t - timestep, num_col))
			goals.push_back(t);
	}
	return goals;
}

// whether the path between loc1 and loc2 is Manhattan-optimal
bool isManhattanOptimal(int loc1, int loc2, int dist, int num_col)
{

	return abs(loc1 / num_col - loc2 / num_col) + abs(loc1 % num_col - loc2 % num_col) == dist;
}

//// whther two rectangle conflicts are idenitical
//bool equalRectangleConflict(const Conflict& c1, const Conflict& c2)
//{
//	if (std::get<2>(c1) != std::get<2>(c2)) //Not same vertex Rg
//		return false;
//	else if ((std::get<0>(c1) == std::get<0>(c2) && std::get<1>(c1) == std::get<1>(c2)) ||
//		(std::get<0>(c1) == std::get<1>(c2) && std::get<1>(c1) == std::get<0>(c2))) // Same set of agents
//		return true;
//	else
//		return false;
//}
//
//// find duplicate rectangle conflicts, used to detect whether a semi-/non-cardinal rectangle conflict is unique
//bool findRectangleConflict(const ICBSNode* curr, const Conflict& conflict)
//{
//	while (curr != NULL)
//	{
//		if (equalRectangleConflict(conflict, *curr->conflict))
//			return true;
//		curr = curr->parent;
//	}
//	return false;
//}


//Compute candidates for GR
std::list<int>  getStartCandidates(const std::vector<PathEntry>& path, int timestep, int dir1, int dir2)
{
	std::list<int> starts;
	int curr = path[timestep].location;
	int t = timestep - 1;
	while (t >= 0) 
	{
		int prev = path[t].location;
		if(curr - prev == dir1 || curr - prev == dir2)
		{
			curr = prev;
			t--;
		}
		else
		{
			break;
		}
	}
	starts.push_back(t + 1);
	return starts;
}

std::list<int>  getGoalCandidates(const std::vector<PathEntry>& path, int timestep, int dir1, int dir2)
{
	std::list<int> goals;
	int curr = path[timestep].location;
	int t = timestep + 1;
	while (t < path.size()) 
	{
		int next = path[t].location;
		if (next - curr == dir1 || next - curr == dir2)
		{
			curr = next;
			t++;
		}
		else
		{
			break;
		}
	}
	goals.push_back(t - 1);
	return goals;
}

template <class Map>
bool ExtractBarriers(const MDD<Map>& mdd, int dir1, int dir2, int start, int goal, int start_time, int num_col, std::list<Constraint>& B)
{
	int num_barrier;
	int barrierZeroTime;
	int sign1 = dir1 / abs(dir1);
	int sign2 = dir2 / abs(dir2);
	if (abs(dir1) == 1) //vertical barriers
	{
		num_barrier = abs(start % num_col - goal % num_col) + 1;
		barrierZeroTime = - start / num_col * sign2 + start_time;
	}
	else
	{
		num_barrier = abs(start / num_col - goal / num_col) + 1;
		barrierZeroTime = - start % num_col * sign2 + start_time;
	}
	std::vector<int> extent_L(num_barrier, INT_MAX);
	std::vector<int> extent_U(num_barrier, -1);
	
	std::unordered_map<MDDNode*, std::vector<bool>> blocking;
	
	MDDNode* n = mdd.levels[0].front();
	std::vector<bool> block(num_barrier, false);
	int hasStart = false;
	if (start_time == 0)
	{
		extent_L[0] = 0;
		extent_U[0] = 0;
		block[0] = true;
		hasStart = true;
	}
	blocking[n] = block;

	for (int t = 1; t < mdd.levels.size(); t++)
	{
		for (auto n : mdd.levels[t])
		{
			std::vector<bool> block(num_barrier, true);
			for (auto parent : n->parents)
			{
				std::vector<bool> parent_block = blocking[parent];
				for (int i = 0; i < num_barrier; i++)
				{
					if(!parent_block[i])
						block[i] = false;
				}
			}
			int barrier_id, barrier_time;
			if (abs(dir1) == 1) //vertical barriers
			{
				barrier_id = abs(start % num_col - n->location % num_col);
				barrier_time = n->location / num_col * sign2 + barrierZeroTime + barrier_id;
			}
			else
			{
				barrier_id = abs(start / num_col - n->location / num_col);
				barrier_time = n->location % num_col * sign2 + barrierZeroTime + barrier_id;
			}
			if (barrier_id < num_barrier && !block[barrier_id] && barrier_time == n->level)
			{
				if (n->children.size() == 1 && extent_L[barrier_id] == INT_MAX &&
					abs(dir1) * abs(n->location - n->children.front()->location) == num_col);// the only child node is on the same barrier
				else
				{
					extent_L[barrier_id] = std::min(extent_L[barrier_id], n->level);
					extent_U[barrier_id] = std::max(extent_U[barrier_id], n->level);
					block[barrier_id] = true;
				}						
			}
			blocking[n] = block;
		}
	}
	
	n = mdd.levels.back().front();
	block = blocking[n];
	for (int i = 0; i < num_barrier; i++)
	{
		if (block[i])
		{
			int barrier_start_x, barrier_start_y, barrier_end_x, barrier_end_y, barrier_end_time;
			if (abs(dir1) == 1) //vertical barriers
			{
				barrier_start_x = (extent_L[i] - barrierZeroTime  - i) * sign2;
				barrier_start_y = start % num_col + i * sign1;
				barrier_end_x = (extent_U[i] - barrierZeroTime - i) * sign2;
				barrier_end_y = barrier_start_y;
			}
			else
			{
				barrier_start_x = start / num_col + i * sign1;
				barrier_start_y = (extent_L[i] - barrierZeroTime - i) * sign2;
				barrier_end_x = barrier_start_x;
				barrier_end_y = (extent_U[i] - barrierZeroTime - i) * sign2;
			}
			barrier_end_time = extent_U[i];
			B.emplace_back(barrier_start_x * num_col + barrier_start_y, 
				barrier_end_x * num_col + barrier_end_y, barrier_end_time, constraint_type::BARRIER);
		}
	}
	return !B.empty();
}

bool isEntryBarrier(const Constraint& b1, const Constraint& b2, int dir1, int num_col)
{

	std::pair<int, int> b1_l = std::make_pair(std::get<0>(b1) / num_col, std::get<0>(b1) % num_col);
	std::pair<int, int> b1_u = std::make_pair(std::get<1>(b1) / num_col, std::get<1>(b1) % num_col);
	std::pair<int, int> b2_l = std::make_pair(std::get<0>(b2) / num_col, std::get<0>(b2) % num_col);
	std::pair<int, int> b2_u = std::make_pair(std::get<1>(b2) / num_col, std::get<1>(b2) % num_col);
	if (dir1 == num_col && b2_l.first >= b1_l.first)
		return true;
	else if (dir1 == -num_col && b2_l.first <= b1_l.first)
		return true;
	else if (dir1 == 1 && b2_l.second >= b1_l.second)
		return true;
	else if (dir1 == -1 && b2_l.second <= b1_l.second)
		return true;
	else 
		return false;
}

bool isExitBarrier(const Constraint& b1, const Constraint& b2, int dir1, int num_col)
{

	std::pair<int, int> b1_l = std::make_pair(std::get<0>(b1) / num_col, std::get<0>(b1) % num_col);
	std::pair<int, int> b1_u = std::make_pair(std::get<1>(b1) / num_col, std::get<1>(b1) % num_col);
	std::pair<int, int> b2_l = std::make_pair(std::get<0>(b2) / num_col, std::get<0>(b2) % num_col);
	std::pair<int, int> b2_u = std::make_pair(std::get<1>(b2) / num_col, std::get<1>(b2) % num_col);

	if (dir1 == num_col && b2_u.first <= b1_l.first)
		return true;
	else if (dir1 == -num_col && b2_u.first >= b1_l.first)
		return true;
	else if (dir1 == 1 && b2_u.second <= b1_l.second)
		return true;
	else if (dir1 == -1 && b2_u.second >= b1_l.second)
		return true;
	else
		return false;
}

std::pair<int, int> getIntersection(const Constraint& b1, const Constraint& b2, int num_col)
{
	std::pair<int, int> b1_l = std::make_pair(std::get<0>(b1) / num_col, std::get<0>(b1) % num_col);
	std::pair<int, int> b1_u = std::make_pair(std::get<1>(b1) / num_col, std::get<1>(b1) % num_col);
	std::pair<int, int> b2_l = std::make_pair(std::get<0>(b2) / num_col, std::get<0>(b2) % num_col);
	std::pair<int, int> b2_u = std::make_pair(std::get<1>(b2) / num_col, std::get<1>(b2) % num_col);

	if (b1_l.first == b1_u.first && b2_l.second == b2_u.second)
		return std::make_pair(b1_l.first, b2_l.second);
	else 
		return std::make_pair(b2_l.first, b1_l.second);
}

void getCorners(const Constraint& b1, const Constraint& b2, int dir1, int dir2, int num_col, 
	std::pair<int,int>& R1, std::pair<int, int>& R2, std::pair<int, int>& Rs, std::pair<int, int>& Rg)
{
	std::pair<int, int> b1_l = std::make_pair(std::get<0>(b1) / num_col, std::get<0>(b1) % num_col);
	std::pair<int, int> b1_u = std::make_pair(std::get<1>(b1) / num_col, std::get<1>(b1) % num_col);
	std::pair<int, int> b2_l = std::make_pair(std::get<0>(b2) / num_col, std::get<0>(b2) % num_col);
	std::pair<int, int> b2_u = std::make_pair(std::get<1>(b2) / num_col, std::get<1>(b2) % num_col);
	
	if (dir1 == 1 || dir2 == 1)
	{
		Rs.first = std::min(b1_l.first, b2_l.first);
		Rg.first = std::max(b1_u.first, b2_u.first);
	}
	else if (dir1 == -1 || dir2 == -1)
	{
		Rs.first = std::max(b1_l.first, b2_l.first);
		Rg.first = std::min(b1_u.first, b2_u.first);
	}
	if (dir1 == num_col || dir2 == num_col)
	{
		Rs.second = std::min(b1_l.second, b2_l.second);
		Rg.second = std::max(b1_u.second, b2_u.second);
	}
	else if (dir1 == -num_col || dir2 == -num_col)
	{
		Rs.second = std::max(b1_l.second, b2_l.second);
		Rg.second = std::min(b1_u.second, b2_u.second);
	}

	if (abs(dir1) == 1)
	{
		R1.first = Rs.first;
		R1.second = Rg.second;
		R2.first = Rg.first;
		R2.second = Rs.second;
	}
	else
	{
		R1.first = Rg.first;
		R1.second = Rs.second;
		R2.first = Rs.first;
		R2.second = Rg.second;
	}
}

bool isCut(const Constraint b, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg, int num_col)
{
	std::pair<int, int> b_l = std::make_pair(std::get<0>(b) / num_col, std::get<0>(b) % num_col);
	std::pair<int, int> b_u = std::make_pair(std::get<1>(b) / num_col, std::get<1>(b) % num_col);
	if (b_l == b_u)
	{
		if (((Rs.first <= b_l.first && b_u.first <= Rg.first)|| (Rs.first >= b_l.first && b_u.first >= Rg.first)) &&
			((Rs.second <= b_l.second && b_u.second <= Rg.second) || (Rs.second >= b_l.second && b_u.second >= Rg.second)))
			return true;
		else
			return false;
	}
	if (Rs.first <= b_l.first && b_l.first <= b_u.first && b_u.first <= Rg.first && b_l.second == b_u.second)
		return true;
	else if (Rs.second <= b_l.second && b_l.second <= b_u.second && b_u.second <= Rg.second && b_l.first == b_u.first)
		return true;
	else if (Rs.first >= b_l.first && b_l.first >= b_u.first && b_u.first >= Rg.first && b_l.second == b_u.second)
		return true;
	else if (Rs.second >= b_l.second && b_l.second >= b_u.second && b_u.second >= Rg.second && b_l.first == b_u.first)
		return true;
	else
		return false;
}

bool blockedNodes(const std::vector<PathEntry>& path, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg, int Rg_t, int dir, int num_col)
{
	int sign;
	std::pair<int, int> b_l;
	if (abs(dir) == 1)
	{
		b_l.first = Rg.first;
		b_l.second = Rs.second;
	}
	else 
	{
		b_l.first = Rs.first;
		b_l.second = Rg.second;
	}
	
	int t_max = std::min(Rg_t, (int)path.size() - 1);
	int t_b_l = Rg_t - abs(b_l.first - Rg.first) - abs(b_l.second - Rg.second);
	int t_min = std::max(0, t_b_l);

	for (int t = t_min; t <= t_max; t++)
	{
		int loc = b_l.first  * num_col + b_l.second + (t - t_b_l) * dir;
		if (path[t].location == loc)
		{
				return true;
		}
	}
	return false;
}

template <class Map>
bool blockedNodes(const MDD<Map>& mdd, const Constraint b, int num_col)
{
	std::pair<int, int> b_l = std::make_pair(std::get<0>(b) / num_col, std::get<0>(b) % num_col);
	std::pair<int, int> b_u = std::make_pair(std::get<1>(b) / num_col, std::get<1>(b) % num_col);
	int t_b_u = std::get<2>(b);
	int t_max = std::min(t_b_u, (int)mdd.levels.size() - 1);
	int t_b_l = t_b_u - abs(b_l.first - b_u.first) - abs(b_l.second - b_u.second);
	int t_min = std::max(0, t_b_l);
	int sign1 = b_u.first - b_l.first;
	int sign2 = b_u.second - b_l.second;
	if (abs(sign1) > 1)
		sign1 /= abs(sign1);
	if (abs(sign2) > 1)
		sign2 /= abs(sign2);
	for (int t = t_min; t <= t_max; t++)
	{
		int loc = (b_l.first + (t - t_b_l) * sign1) * num_col + b_l.second + (t - t_b_l) * sign2;
		for (auto n : mdd.levels[t])
		{
			if (n->location == loc)
				return true;
		}
	}
	return false;
}

template <class Map>
void generalizedRectangle(const std::vector<PathEntry>& path1, const std::vector<PathEntry>& path2, const MDD<Map>& mdd1, const MDD<Map>& mdd2, 
	const std::list<Constraint>::const_iterator& b1, const std::list<Constraint>::const_iterator& b2,
	const std::list<Constraint>& B1, const std::list<Constraint>& B2, int timestep, int num_col, 
	int& best_type, std::pair<int, int>& best_Rs, std::pair<int, int>& best_Rg, int time_limit, std::set<std::pair<int, int>> &visitedRs)
{
	if (std::clock() > time_limit)
		return;
	int loc = path1[timestep].location;
	int dir1 = loc - path1[timestep - 1].location;
	int dir2 = loc - path2[timestep - 1].location;

	std::list<Constraint>::const_iterator b1_entry = b1;
	std::list<Constraint>::const_iterator b2_entry = b2;

	if (best_type == 2)
		return;
	while (b1_entry != B1.cend() && b2_entry != B2.cend())
	{
		if (!isEntryBarrier(*b1_entry, *b2_entry, dir1, num_col))
		{
			++b2_entry;
			continue;
		}
		if (!isEntryBarrier(*b2_entry, *b1_entry, dir2, num_col))
		{
			++b1_entry;
			continue;
		}
		//break;
	//}
	//if (b1_entry != B1.cend() &&  b2_entry != B2.cend())
	//{
		std::pair<int, int> Rs = getIntersection(*b1_entry, *b2_entry, num_col);
		auto it = visitedRs.find(Rs);
		if (it != visitedRs.end())
			return;
		visitedRs.insert(Rs);
		std::list<Constraint>::const_reverse_iterator  b1_exit = B1.rbegin();
		std::list<Constraint>::const_reverse_iterator  b2_exit = B2.rbegin();
		bool move1 = true;
		while (b1_exit != B1.rend() && b2_exit != B2.rend())
		{
			if (!isExitBarrier(*b1_exit, *b2_entry, dir1, num_col))
			{
				move1 = false;
				break;
			}
			if (!isExitBarrier(*b2_exit, *b1_entry, dir2, num_col))
			{
				move1 = true;
				break;
			}
			std::pair<int, int> Rg = getIntersection(*b1_exit, *b2_exit, num_col); 
			int Rg_t = timestep + std::abs(Rg.first - loc / num_col) + std::abs(Rg.second - loc % num_col);
			if (!blockedNodes(path1, Rs, Rg, Rg_t, dir2, num_col))
			{
				++b1_exit;
				continue;
			}
			if (!blockedNodes(path2, Rs, Rg, Rg_t, dir1, num_col))
			{
				++b2_exit;
				continue;
			}

			/*if (!blockedNodes<Map>(mdd1, *b1_exit, num_col))
			{
				++b1_exit;
				continue;
			}
			if (!blockedNodes<Map>(mdd2, *b2_exit, num_col))
			{
				++b2_exit;
				continue;
			}*/
				
			bool cut1 = isCut(*b1_exit, Rs, Rg, num_col);
			bool cut2 = isCut(*b2_exit, Rs, Rg, num_col);
			int type = (int)(cut1)+(int)(cut2);
			if (type > best_type)
			{
				best_Rs = Rs;
				best_Rg = Rg;
				best_type = type;
				if (best_type == 2)
					return;
			}

			if (!cut1)
				++b1_exit;
			else if (!cut2)
				++b2_exit;
		}
		if (b1_exit == B1.rend())
		{
			++b2_entry;
		}
		else if (b2_exit == B2.rend())
		{
			++b1_entry;
		}
		/*else if (move1)
		{
			++b1_entry;
		}
		else
		{
			++b2_entry;
		}*/
		std::list<Constraint>::const_iterator b = b1_entry;
		b++;
		//std::cout << *b << " " << *b2_entry << std::endl;
		generalizedRectangle(path1, path2, mdd1, mdd2, b, b2_entry, B1, B2, timestep, num_col,
			best_type, best_Rs, best_Rg, time_limit, visitedRs);
		b = b2_entry;
		b++;
		//std::cout << *b1_entry << " " << *b << std::endl;
		generalizedRectangle(path1, path2, mdd1, mdd2, b1_entry, b, B1, B2, timestep, num_col,
			best_type, best_Rs, best_Rg, time_limit, visitedRs);
		return;
		
	}
	return;
}

template void generalizedRectangle<MapLoader>(const std::vector<PathEntry>& path1, const std::vector<PathEntry>& path2, const MDD<MapLoader>& mdd1, const MDD<MapLoader>& mdd2,
	const std::list<Constraint>::const_iterator& b1, const std::list<Constraint>::const_iterator& b2,
	const std::list<Constraint>& B1, const std::list<Constraint>& B2, int timestep, int num_col,
	int& best_type, std::pair<int, int>& best_Rs, std::pair<int, int>& best_Rg, int time_limit, std::set<std::pair<int, int>> &visitedRs);
template bool blockedNodes<MapLoader>(const MDD<MapLoader>& mdd, const Constraint b, int num_col);
template bool ExtractBarriers<MapLoader>(const MDD<MapLoader>& mdd, int dir1, int dir2, int start, int goal, int start_time, int num_col, std::list<Constraint>& B);

//template void generalizedRectangle<FlatlandLoader>(const std::vector<PathEntry>& path1, const std::vector<PathEntry>& path2, const MDD<FlatlandLoader>& mdd1, const MDD<FlatlandLoader>& mdd2,
//	const std::list<Constraint>::const_iterator& b1, const std::list<Constraint>::const_iterator& b2,
//	const std::list<Constraint>& B1, const std::list<Constraint>& B2, int timestep, int num_col,
//	int& best_type, std::pair<int, int>& best_Rs, std::pair<int, int>& best_Rg, int time_limit, std::set<std::pair<int, int>> &visitedRs);
//template bool blockedNodes<FlatlandLoader>(const MDD<FlatlandLoader>& mdd, const Constraint b, int num_col);
//template bool ExtractBarriers<FlatlandLoader>(const MDD<FlatlandLoader>& mdd, int dir1, int dir2, int start, int goal, int start_time, int num_col, std::list<Constraint>& B);
