#pragma once


#include "LLNode.h"
#include <memory>
#include "common.h"
#include <string>
#include <sstream>
#include "MDDNode.h"
using namespace std;




// <loc, -1, t, VERTEX>
// <from, to, t, EDGE> 
// <B1, B2, t, RECTANGLE>
// <loc, t1, t2, CORRIDOR> 
// <loc, agent_id, t, TARGET>: path of agent_id should be of length at most t, and any other agent cannot be at loc at or after timestep t
// <-1, agent_id, t>: path of agent_id should be of length at least t + 1 


std::ostream& operator<<(std::ostream& os, const Constraint& constraint);


// add a pair of barrier constraints
void addBarrierConstraints(int S1, int S2, int S1_t, int S2_t, int Rg, int num_col,
	std::list<std::tuple<int, int, int>>& constraints1, std::list<std::tuple<int, int, int>>& constraints2);

// add a pair of modified barrier constraints
bool addModifiedBarrierConstraints(const std::vector<PathEntry>& path1, const std::vector<PathEntry>& path2,
	int S1_t, int S2_t, int Rg, int num_col,
	std::list<std::tuple<int, int, int>>& constraints1, std::list<std::tuple<int, int, int>>& constraints2);

// add a horizontal modified barrier constraint
bool addModifiedHorizontalBarrierConstraint(const std::vector<PathEntry>& path, int x,
	int Ri_y, int Rg_y, int Rg_t, int num_col,
	std::list<Constraint>& constraints);

// add a vertival modified barrier constraint
bool addModifiedVerticalBarrierConstraint(const std::vector<PathEntry>& path, int y,
	int Ri_x, int Rg_x, int Rg_t, int num_col,
	std::list<Constraint>& constraints);

bool addKDelayBarrierConstraints(int S1, int S2, int S1_t, int S2_t, pair<int, int> Rs, pair<int, int> Rg, int G1, int G2, int num_col,
	std::vector<std::list<Constraint>>& multiConstraint1, std::vector<std::list<Constraint>>& multiConstraint2,
	int k, int RM4way);

// add a vertival modified barrier constraint
bool addModifiedVerticalLongBarrierConstraint(const std::vector<PathEntry>& path, int y,
	int Ri_x, int Rg_x, int Rg_t, int num_col, int St,
	std::list<Constraint>& constraints, int k, const MDDLevels* kMDD);

// add a horizontal modified barrier constraint
bool addModifiedHorizontalLongBarrierConstraint(const std::vector<PathEntry>& path, int x,
	int Ri_y, int Rg_y, int Rg_t, int num_col, int St,
	std::list<Constraint>& constraints, int k, const MDDLevels* kMDD);

// add a vertival modified barrier constraint for 4 way split
bool add4WayModifiedVerticalLongBarrierConstraint(const std::vector<PathEntry>& path, int y,
	int Ri_x, int Rg_x, int Rg_t, int num_col, int St,
	std::list<Constraint>& constraints, int k);

// add a horizontal modified barrier constraint for 4 way split
bool add4WayModifiedHorizontalLongBarrierConstraint(const std::vector<PathEntry>& path, int x,
	int Ri_y, int Rg_y, int Rg_t, int num_col, int St,
	std::list<Constraint>& constraints, int k);

bool addFlippedVerticalLongBarrierConstraint(const std::vector<PathEntry>& path, int y,
	vector<int> vertical, vector<int> verticalMin, vector<int> verticalMax, int num_col, int St,
	std::list<Constraint>& constraints, int k, MDDPath* kMDD);

bool addFlippedHorizontalLongBarrierConstraint(const std::vector<PathEntry>& path, int x,
	vector<int> horizontal, vector<int> horizontalMin, vector<int> horizontalMax, int num_col, int St,
	std::list<Constraint>& constraints, int k, MDDPath* kMDD);

bool addGeneralKVerticalBarrierConstraint(const std::vector<PathEntry>& path, int y,
                                          int Ri_x, int Rg_x, int Rg_t, int num_col, int St,
                                          std::list<Constraint>& constraints, int k, const MDDLevels* kMDD);

bool addGeneralKHorizontalBarrierConstraint(const std::vector<PathEntry>& path, int x,
                                            int Ri_y, int Rg_y, int Rg_t, int num_col, int St,
                                            std::list<Constraint>& constraints, int k, const MDDLevels* kMDD);




class Conflict
{
public:

	int a1;
	int a2;
	int t;
	int k=0;
	int s1;
	int s2;
	int g1;
	int g2;
	int s1_t;
	int s2_t;
	int g1_t;
	int g2_t;
	int rs=0;
	int rg=0;
	int t_sg;
	int originalConf1=0;
	int originalConf2=-1;
	int flipType = 0;
	bool repeat = false;
	bool isChasing = false;
	std::list<Constraint> constraint1;
	std::vector<std::list<Constraint>> multiConstraint1;
	std::list<Constraint> constraint2;
	std::vector<std::list<Constraint>> multiConstraint2;

	conflict_type type;
	conflict_priority p = conflict_priority::UNKNOWN;

	Conflict() {};
	Conflict(int v,int k,int t) {
		this->originalConf1 = v;
		this->originalConf2 = -1;
		this->k = k;
		this->t = t;
	};

	void vertexConflict(int a1, int a2, int v, int t,int k=0,int kRobust =0)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->k = k;
		this->originalConf1 = v;
		this->originalConf2 = -1;
		for (int i = 0; i <= kRobust; i++) {
			this->constraint1.emplace_back(v, -1, t+i, constraint_type::VERTEX);
			this->constraint2.emplace_back(v, -1, t+i, constraint_type::VERTEX);
		}
		type = conflict_type::STANDARD;
	}
		
	void edgeConflict(int a1, int a2, int v1, int v2, int t)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->k = 0;
		this->originalConf1 = v1;
		this->originalConf2 = v2;

		this->constraint1.emplace_back(v1, v2, t, constraint_type::EDGE);
		this->constraint2.emplace_back(v2, v1, t, constraint_type::EDGE);
		type = conflict_type::STANDARD;
	}

	void trainCorridorConflict(int a1, int a2, int v1, int v2, int t1, int t2, int e1, int e2, int k, int kRobust)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->k = k;
		this->t = std::min(e1, e2);
		this->originalConf1 = v1;
		this->originalConf2 = v2;
		this->constraint1.emplace_back(v1, t2, e2-1 + kRobust, constraint_type::RANGE);
		this->constraint2.emplace_back(v2, t1, e1-1 + kRobust, constraint_type::RANGE);
		type = conflict_type::CORRIDOR2;
	}

	// t3 
	void corridorConflict(int a1, int a2, int v1, int v2, int t3, int t4, int t3_, int t4_, int k,int kRobust)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->k = k;
		this->t = std::min(t3, t4);
		this->originalConf1 = v1;
		this->originalConf2 = v2;
		//k is corridor length
		this->constraint1.emplace_back(v1, t3, std::min(t3_ - 1 , t4 + k) + kRobust, constraint_type::RANGE);
		this->constraint2.emplace_back(v2, t4, std::min(t4_ - 1 , t3 + k) + kRobust, constraint_type::RANGE);
		type = conflict_type::CORRIDOR2;
	}


	void corridorConflict(int a1, int a2, int v1, int v2, int t1, int t2, int k, int h, int kRobust)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = std::min(t1, t2);
		this->originalConf1 = v1;
		this->originalConf2 = v2;
		this->k = k;
		this->constraint1.emplace_back(v1, t1, t1 + 2 * k - 1 + kRobust, constraint_type::RANGE);
		this->constraint1.emplace_back(v2, t1 + k, std::min(t2 + 2 * k, t1 + h - 1) + kRobust, constraint_type::RANGE);
		this->constraint2.emplace_back(v2, t2, t2 + 2 * k - 1 + kRobust, constraint_type::RANGE);
		this->constraint2.emplace_back(v1, t2 + k, std::min(t1 + 2 * k, t2 + h - 1) + kRobust, constraint_type::RANGE);
		type = conflict_type::CORRIDOR4;
	}

	bool rectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg, int move1, int move2, int Rg_t, const std::vector<Path*>& paths, int num_col) // For GR
	{
		this->a1 = a1;
		this->a2 = a2;
		this->k = 0;
		this->t_sg = Rg_t - abs(Rg.first - Rs.first) - abs(Rg.second - Rs.second);
		this->rs = Rs.first*num_col + Rs.second;
		this->rg = Rg.first*num_col + Rg.second;;
		if (abs(move1) == 1 || abs(move2) > 1) // first agent moves horizontally and second agent moves vertically
		{
			if (!addModifiedVerticalBarrierConstraint(*paths[a1], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint1))
			{
				return false;
			}
			if (!addModifiedHorizontalBarrierConstraint(*paths[a2], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint2))
			{
				return false;
			}
		}
		else
		{
			if (!addModifiedHorizontalBarrierConstraint(*paths[a1], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint1))
			{
				return false;
			}
			if (!addModifiedVerticalBarrierConstraint(*paths[a2], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint2))
			{
				return false;
			}
		}
		type = conflict_type::RECTANGLE;
		return true;
	}

	bool rectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
		const std::pair<int, int>& s1, const std::pair<int, int>& s2, int Rg_t, const std::vector<Path*>& paths, int num_col) // For RM
	{
		this->a1 = a1;
		this->a2 = a2;
		this->k = 0;
		this->t_sg = Rg_t - abs(Rg.first - Rs.first) - abs(Rg.second - Rs.second);
		this->rs = Rs.first*num_col + Rs.second;
		this->rg = Rg.first*num_col + Rg.second;;
		if (s1.first == s2.first)
		{
			if ((s1.second - s2.second) * (s2.second - Rg.second) >= 0)
			{
				// first agent moves horizontally and second agent moves vertically
				if (!addModifiedVerticalBarrierConstraint(*paths[a1], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint1))
				{
					return false;
				}
				if (!addModifiedHorizontalBarrierConstraint(*paths[a2], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint2))
				{
					return false;
				}
			}
			else
			{
				// first agent moves vertically and second agent moves horizontally
				if (!addModifiedHorizontalBarrierConstraint(*paths[a1], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint1))
				{
					return false;
				}
				if (!addModifiedVerticalBarrierConstraint(*paths[a2], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint2))
				{
					return false;
				}
			}
		}
		else if ((s1.first - s2.first)*(s2.first - Rg.first) >= 0)
		{
			// first agent moves vertically and second agent moves horizontally
			if (!addModifiedHorizontalBarrierConstraint(*paths[a1], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint1))
			{
				return false;
			}
			if (!addModifiedVerticalBarrierConstraint(*paths[a2], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint2))
			{
				return false;
			}
		}
		else
		{
			// first agent moves horizontally and second agent moves vertically
			if (!addModifiedVerticalBarrierConstraint(*paths[a1], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint1))
			{
				return false;
			}
			if (!addModifiedHorizontalBarrierConstraint(*paths[a2], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint2))
			{
				return false;
			}
		}
		type = conflict_type::RECTANGLE;
		return true;
	}


	bool kRectangleConflict(int a1, int a2, int S1, int S2, int S1_t, int S2_t, pair<int, int> Rs, pair<int, int> Rg,
		int Rg_t, int G1, int G2, int num_col,int k, int RM4way) // For CR and R
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t_sg = Rg_t - abs(Rg.first - Rs.first) - abs(Rg.second - Rs.second);
		this->rs = Rs.first*num_col + Rs.second;
		this->rg = Rg.first*num_col + Rg.second;
		if (addKDelayBarrierConstraints(S1, S2, S1_t, S2_t, Rs, Rg, G1, G2, num_col,
			multiConstraint1, multiConstraint2, k, RM4way)) {
			type = conflict_type::RECTANGLE4;
			return true;

		}
		else
			return false;

		
	}

	bool kRectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
		const std::pair<int, int>& s1, const std::pair<int, int>& s2, int rt1,int rt2,
		const std::vector<Path*>& paths, int S1_t, int S2_t, const std::pair<int, int>& G1, const std::pair<int, int>& G2,
		int num_col, int k, int RM4way, bool I_RM = false, const MDDLevels* a1kMDD = NULL, const MDDLevels* a2kMDD = NULL) // For K-RM
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t_sg = rt1;
		this->rs = Rs.first*num_col + Rs.second;
		this->rg = Rg.first*num_col + Rg.second;

		int s1_x = s1.first;
		int s1_y = s1.second;
		int s2_x = s2.first;
		int s2_y = s2.second;
		int Rg_x = Rg.first;
		int Rg_y = Rg.second;
		int g1_x = G1.first;
		int g1_y = G1.second;
		int g2_x = G2.first;
		int g2_y = G2.second;

//		if (RM4way >=2  && RM4way <= 5) {
//			if (k > 1) {
//				k = 1;
//			}
//		}

		bool split4way = false;
		bool a1_4way=false;
		bool a2_4way=false;
		bool no_mdd_check = false;

		if (RM4way==-2 || RM4way == 3 || RM4way == 5 || RM4way ==7) {
			split4way = true;
		}
		if (RM4way >= 4) {
			no_mdd_check = true;
		}

		int a1Rg = S1_t + getMahattanDistance(s1_x, s1_y, Rg_x, Rg_y);
		int a1RgBypass = a1Rg + 2 * (getMahattanDistance(s2_x, s2_y, Rs.first, Rs.second) + 1);

		int a2Rg = S2_t + getMahattanDistance(s2_x, s2_y, Rg_x, Rg_y);
		int a2RgBypass = a2Rg + 2 * (getMahattanDistance(s1_x, s1_y, Rs.first, Rs.second) + 1);

		if (RM4way ==1||RM4way ==6) {
			a2_4way = a2RgBypass <= a1Rg + k;//only need to compare with a1rg, because time range on Rg is based on the root time of a1
			a1_4way = a1RgBypass <= a1Rg + k;
		}

		if (RM4way == -1 && (a1_4way || a2_4way)) {
			return false;
		}

		int extended = k / 2;
		int R1_x, R1_y, R2_x, R2_y, G1_x, G1_y, G2_x, G2_y, G1_t, G2_t,E1_t,E2_t;

		if ((RM4way ==5 || I_RM) && (
			(((s2_x - s1_x) * (s1_x - g1_x) < 0 && (s2_y - s1_y) * (s1_y - g1_y) < 0) && ((s1_x - g1_x) * (g1_x - g2_x) > 0 || (s1_y - g1_y) * (g1_y - g2_y) < 0))
			||
			(((s1_x - s2_x) * (s2_x - g2_x) < 0 && (s1_y - s2_y) * (s2_y - g2_y) < 0) && ((s2_x - g2_x) * (g2_x - g1_x) < 0 || (s2_y - g2_y) * (g2_y - g1_y) > 0))
			)) { // s1 always in the middle,s2 always between s1 and g1 && g1 lies right side of s1 s2 g2 line
			// or s2 always in the middle, s1 always between s2 and g2 && g2 lies left side of s2 s1 g1 line
			int sign_a1 = g1_y - s1_y >= 0 ? 1 : -1;
			int sign_a2 = g2_x - s2_x >= 0 ? 1 : -1;

			int a1_exit = Rg_y + sign_a1 * extended;
			a1_exit = sign_a1 * a1_exit < sign_a1*g1_y ? a1_exit : g1_y;
//			int a1_entrance = Rs.second - sign_a1 * extended;
//			a1_entrance = sign_a1 * a1_entrance > sign_a1*s1_y ? a1_entrance : s1_y;


			int a2_exit = Rg_x + sign_a2 * extended;
			a2_exit = sign_a2 * a2_exit < sign_a2*g2_x ? a2_exit : g2_x;
//			int a2_entrance = Rs.first - +sign_a2 * extended;
//			a2_entrance = sign_a2 * a2_entrance > sign_a2 * s2_x ? a2_entrance : s2_x;

            int a1_entrance = Rs.second;
            int a2_entrance = Rs.first;


			R1_x = Rs.first;
			G1_x = Rg_x;

			R2_y = Rs.second;
			G2_y = Rg_y;

			G1_t = rt1 + getMahattanDistance(Rs.first, Rs.second, G1_x, a1_exit);
			E1_t = rt1 + getMahattanDistance(Rs.first, a1_entrance, G1_x, a1_entrance);
			G2_t = rt2 + getMahattanDistance(Rs.first, Rs.second, a2_exit,G2_y);
			E2_t = rt2 + getMahattanDistance(a2_entrance, Rs.second, a2_entrance, G2_y);


			std::list<Constraint> constraint11;
			if (no_mdd_check)
				add4WayModifiedVerticalLongBarrierConstraint(*paths[a1], a1_exit, R1_x, G1_x, G1_t, num_col, S1_t, constraint11, k);
			else
				addModifiedVerticalLongBarrierConstraint(*paths[a1], a1_exit, R1_x, G1_x, G1_t, num_col, S1_t, constraint11, k, a1kMDD);
			multiConstraint1.push_back(constraint11);


			//chasing case always 4 way split
			std::list<Constraint> constraint12;
			if (no_mdd_check)
				add4WayModifiedVerticalLongBarrierConstraint(*paths[a1], a1_entrance, R1_x, G1_x, E1_t, num_col, S1_t, constraint12, k);
			else
				addModifiedVerticalLongBarrierConstraint(*paths[a1], a1_entrance, R1_x, G1_x, E1_t, num_col, S1_t, constraint12, k, a1kMDD);
			multiConstraint1.push_back(constraint12);


			std::list<Constraint> constraint21;
			if (no_mdd_check)
				add4WayModifiedHorizontalLongBarrierConstraint(*paths[a2], a2_exit, R2_y, G2_y, G2_t, num_col, S2_t, constraint21, k);
			else
				addModifiedHorizontalLongBarrierConstraint(*paths[a2], a2_exit, R2_y, G2_y, G2_t, num_col, S2_t, constraint21, k, a2kMDD);
			multiConstraint2.push_back(constraint21);

			//chasing case always 4 way split
			std::list<Constraint> constraint22;
			if (no_mdd_check)
				add4WayModifiedHorizontalLongBarrierConstraint(*paths[a2], a2_entrance, R2_y, G2_y, E2_t, num_col, S2_t, constraint22, k);
			else
				addModifiedHorizontalLongBarrierConstraint(*paths[a2], a2_entrance, R2_y, G2_y, E2_t, num_col, S2_t, constraint22, k, a2kMDD);
			multiConstraint2.push_back(constraint22);
			this->isChasing = true;


		}
		else if ((RM4way ==5 || I_RM) && (
			(((s1_x - s2_x) * (s2_x - g2_x) < 0 && (s1_y - s2_y) * (s2_y - g2_y) < 0) && ((s2_x - g2_x) * (g2_x - g1_x) > 0 || (s2_y - g2_y) * (g2_y - g1_y) < 0))
			||
			(((s2_x - s1_x) * (s1_x - g1_x) < 0 && (s2_y - s1_y) * (s1_y - g1_y) < 0) && ((s1_x - g1_x) * (g1_x - g2_x) < 0 || (s1_y - g1_y) * (g1_y - g2_y) > 0))
			)) 
		{ 
			// s2 always in the middle, s1 always between s2 and g2 && g2 lies right side of s2 s1 g1 line
			//or s1 always in the middle,s2 always between s1 and g1 && g1 lies left side of s1 s2 g2 line

			int sign_a1 = g1_x - s1_x >= 0 ? 1 : -1;
			int sign_a2 = g2_y - s2_y >= 0 ? 1 : -1;

			int a1_exit = Rg_x + sign_a1 * extended;
			a1_exit = sign_a1 * a1_exit < sign_a1*g1_x ? a1_exit : g1_x;
//			int a1_entrance = Rs.first - sign_a1 * extended;
//			a1_entrance = sign_a1 * a1_entrance > sign_a1*s1_x ? a1_entrance : s1_x;


			int a2_exit = Rg_y + sign_a2 * extended;
			a2_exit = sign_a2 * a2_exit < sign_a2*g2_y ? a2_exit : g2_y;
//			int a2_entrance = Rs.second - +sign_a2 * extended;
//			a2_entrance = sign_a2 * a2_entrance > sign_a2 * s2_y ? a2_entrance : s2_y;

            int a1_entrance = Rs.first;
            int a2_entrance = Rs.second;

			R2_x = Rs.first;
			G2_x = Rg_x;

			R1_y = Rs.second;
			G1_y = Rg.second;


			G1_t = rt1 + getMahattanDistance(Rs.first, Rs.second, a1_exit, G1_y);
			E1_t = rt1 + getMahattanDistance(a1_entrance, Rs.second, a1_entrance, G1_y);
			G2_t = rt2 + getMahattanDistance(Rs.first, Rs.second, G2_x, a2_exit);
			E2_t = rt2 + getMahattanDistance(Rs.first, a2_entrance, G2_x, a2_entrance);


			std::list<Constraint> constraint11;

			if (no_mdd_check)
				add4WayModifiedHorizontalLongBarrierConstraint(*paths[a1], a1_exit, R1_y, G1_y, G1_t, num_col, S1_t, constraint11, k);
			else
				addModifiedHorizontalLongBarrierConstraint(*paths[a1], a1_exit, R1_y, G1_y, G1_t, num_col, S1_t, constraint11, k, a1kMDD);
			multiConstraint1.push_back(constraint11);

			//chasing case always 4 way split
			std::list<Constraint> constraint12;
			if (no_mdd_check)
				add4WayModifiedHorizontalLongBarrierConstraint(*paths[a1], a1_entrance, R1_y, G1_y, E1_t, num_col, S1_t, constraint12, k);
			else
				addModifiedHorizontalLongBarrierConstraint(*paths[a1], a1_entrance, R1_y, G1_y, E1_t, num_col, S1_t, constraint12, k, a1kMDD);
			multiConstraint1.push_back(constraint12);




			std::list<Constraint> constraint21;
			if (no_mdd_check)
				add4WayModifiedVerticalLongBarrierConstraint(*paths[a2], a2_exit, R2_x, G2_x, G2_t, num_col, S2_t, constraint21, k);
			else
				addModifiedVerticalLongBarrierConstraint(*paths[a2], a2_exit, R2_x, G2_x, G2_t, num_col, S2_t, constraint21, k, a2kMDD);
			multiConstraint2.push_back(constraint21);

			//chasing case always 4 way split
			std::list<Constraint> constraint22;
			if (no_mdd_check)
				add4WayModifiedVerticalLongBarrierConstraint(*paths[a2], a2_entrance, R2_x, G2_x, E2_t, num_col, S2_t, constraint22, k);
			else
				addModifiedVerticalLongBarrierConstraint(*paths[a2], a2_entrance, R2_x, G2_x, E2_t, num_col, S2_t, constraint22, k, a2kMDD);
			multiConstraint2.push_back(constraint22);

			this->isChasing = true;


		}
		else if ((s1_x == s2_x && (s1_y - s2_y) * (s2_y - Rg_y) < 0) ||
			(s1_x != s2_x && (s1_x - s2_x)*(s2_x - Rg_x) >= 0))
		{

			
			int sign_a1 = g1_x - s1_x >= 0 ? 1 : -1;
			int sign_a2 = g2_y - s2_y >= 0 ? 1 : -1;

			int a1_exit = Rg_x + sign_a1 * extended;
			a1_exit = sign_a1 * a1_exit < sign_a1*g1_x ? a1_exit : g1_x;
//			int a1_entrance = Rs.first - sign_a1 * extended;
//			a1_entrance = sign_a1 * a1_entrance > sign_a1*s1_x ? a1_entrance : s1_x;


			int a2_exit = Rg_y + sign_a2 * extended;
			a2_exit = sign_a2 * a2_exit < sign_a2*g2_y ? a2_exit : g2_y;
//			int a2_entrance = Rs.second - +sign_a2 * extended;
//			a2_entrance = sign_a2 * a2_entrance > sign_a2 * s2_y ? a2_entrance : s2_y;

            int a1_entrance = Rs.first;
            int a2_entrance = Rs.second;


			R2_x = Rs.first;
			G2_x = Rg_x;

			R1_y = Rs.second;
			G1_y = Rg.second;


			G1_t = rt1 + getMahattanDistance(Rs.first, Rs.second, a1_exit, G1_y);
			E1_t = rt1 + getMahattanDistance(a1_entrance, Rs.second, a1_entrance, G1_y);
			G2_t = rt2 + getMahattanDistance(Rs.first, Rs.second, G2_x, a2_exit);
			E2_t = rt2 + getMahattanDistance(Rs.first, a2_entrance, G2_x, a2_entrance);


			std::list<Constraint> constraint11;

			if (no_mdd_check)
				add4WayModifiedHorizontalLongBarrierConstraint(*paths[a1], a1_exit, R1_y, G1_y, G1_t, num_col, S1_t, constraint11, k);
			else
				addModifiedHorizontalLongBarrierConstraint(*paths[a1], a1_exit, R1_y, G1_y, G1_t, num_col, S1_t, constraint11, k, a1kMDD);
			multiConstraint1.push_back(constraint11);

			if (split4way || a1_4way) {
				std::list<Constraint> constraint12;
				if (no_mdd_check)
					add4WayModifiedHorizontalLongBarrierConstraint(*paths[a1], a1_entrance, R1_y, G1_y, E1_t, num_col, S1_t, constraint12, k);
				else
					addModifiedHorizontalLongBarrierConstraint(*paths[a1], a1_entrance, R1_y, G1_y, E1_t, num_col, S1_t, constraint12, k, a1kMDD);
				multiConstraint1.push_back(constraint12);
			}



			std::list<Constraint> constraint21;
			if (no_mdd_check)
				add4WayModifiedVerticalLongBarrierConstraint(*paths[a2], a2_exit, R2_x, G2_x, G2_t, num_col, S2_t, constraint21, k);
			else
				addModifiedVerticalLongBarrierConstraint(*paths[a2], a2_exit, R2_x, G2_x, G2_t, num_col, S2_t, constraint21, k, a2kMDD);
			multiConstraint2.push_back(constraint21);

			if (split4way || a2_4way) {
				std::list<Constraint> constraint22;
				if (no_mdd_check)
					add4WayModifiedVerticalLongBarrierConstraint(*paths[a2], a2_entrance, R2_x, G2_x, E2_t, num_col, S2_t, constraint22, k);
				else
					addModifiedVerticalLongBarrierConstraint(*paths[a2], a2_entrance, R2_x, G2_x, E2_t, num_col, S2_t, constraint22, k, a2kMDD);
				multiConstraint2.push_back(constraint22);
			}
			



		}
		else
		{

			int sign_a1 = g1_y - s1_y >= 0 ? 1 : -1;
			int sign_a2 = g2_x - s2_x >= 0 ? 1 : -1;

			int a1_exit = Rg_y + sign_a1 * extended;
			a1_exit = sign_a1 * a1_exit < sign_a1*g1_y ? a1_exit : g1_y;
//			int a1_entrance = Rs.second - sign_a1 * extended;
//			a1_entrance = sign_a1 * a1_entrance > sign_a1*s1_y ? a1_entrance : s1_y;


			int a2_exit = Rg_x + sign_a2 * extended;
			a2_exit = sign_a2 * a2_exit < sign_a2*g2_x ? a2_exit : g2_x;
//			int a2_entrance = Rs.first - +sign_a2 * extended;
//			a2_entrance = sign_a2 * a2_entrance > sign_a2 * s2_x ? a2_entrance : s2_x;

            int a1_entrance = Rs.second;
            int a2_entrance = Rs.first;


			R1_x = Rs.first;
			G1_x = Rg_x;

			R2_y = Rs.second;
			G2_y = Rg_y;

			G1_t = rt1 + getMahattanDistance(Rs.first, Rs.second, G1_x, a1_exit);
			E1_t = rt1 + getMahattanDistance(Rs.first, a1_entrance, G1_x, a1_entrance);
			G2_t = rt2 + getMahattanDistance(Rs.first, Rs.second, a2_exit, G2_y);
			E2_t = rt2 + getMahattanDistance(a2_entrance, Rs.second, a2_entrance, G2_y);


			std::list<Constraint> constraint11;
			if (no_mdd_check)
				add4WayModifiedVerticalLongBarrierConstraint(*paths[a1], a1_exit, R1_x, G1_x, G1_t, num_col, S1_t, constraint11, k);
			else
				addModifiedVerticalLongBarrierConstraint(*paths[a1], a1_exit, R1_x, G1_x, G1_t, num_col, S1_t, constraint11, k, a1kMDD);
			multiConstraint1.push_back(constraint11);


			if (split4way || a1_4way) {
				std::list<Constraint> constraint12;
				if (no_mdd_check)
					add4WayModifiedVerticalLongBarrierConstraint(*paths[a1], a1_entrance, R1_x, G1_x, E1_t, num_col, S1_t, constraint12, k);
				else
					addModifiedVerticalLongBarrierConstraint(*paths[a1], a1_entrance, R1_x, G1_x, E1_t, num_col, S1_t, constraint12, k, a1kMDD);
				multiConstraint1.push_back(constraint12);
			}


			std::list<Constraint> constraint21;
			if (no_mdd_check)
				add4WayModifiedHorizontalLongBarrierConstraint(*paths[a2], a2_exit, R2_y, G2_y, G2_t, num_col, S2_t, constraint21, k);
			else
				addModifiedHorizontalLongBarrierConstraint(*paths[a2], a2_exit, R2_y, G2_y, G2_t, num_col, S2_t, constraint21, k, a2kMDD);
			multiConstraint2.push_back(constraint21);

			if (split4way || a2_4way) {
				std::list<Constraint> constraint22;
				if (no_mdd_check)
					add4WayModifiedHorizontalLongBarrierConstraint(*paths[a2], a2_entrance, R2_y, G2_y, E2_t, num_col, S2_t, constraint22, k);
				else
					addModifiedHorizontalLongBarrierConstraint(*paths[a2], a2_entrance, R2_y, G2_y, E2_t, num_col, S2_t, constraint22, k, a2kMDD);
				multiConstraint2.push_back(constraint22);
			}
			

		

			//exit(0);
		}
		type = conflict_type::RECTANGLE4;
		return true;
	}

    int generalKRectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
                            const std::pair<int, int>& s1, const std::pair<int, int>& s2, int rt1,int rt2,
                            const std::vector<Path*>& paths, int S1_t, int S2_t, const std::pair<int, int>& G1, const std::pair<int, int>& G2,
                            int num_col, int a1k, int a2k, int conflict_k, int RM4way, const MDDLevels* a1kMDD, const MDDLevels* a2kMDD) // For K-RM
    {
        this->a1 = a1;
        this->a2 = a2;
        this->t_sg = rt1;
        this->rs = Rs.first*num_col + Rs.second;
        this->rg = Rg.first*num_col + Rg.second;

        int s1_x = s1.first;
        int s1_y = s1.second;
        int s2_x = s2.first;
        int s2_y = s2.second;
        int Rg_x = Rg.first;
        int Rg_y = Rg.second;
        int g1_x = G1.first;
        int g1_y = G1.second;
        int g2_x = G2.first;
        int g2_y = G2.second;

        bool split4way = true;
        bool a1_4way=false;
        bool a2_4way=false;
        bool no_mdd_check = false;


        int a1Rg = S1_t + getMahattanDistance(s1_x, s1_y, Rg_x, Rg_y);
        int a1RgBypass = a1Rg + 2 * (getMahattanDistance(s2_x, s2_y, Rs.first, Rs.second) + 1);

        int a2Rg = S2_t + getMahattanDistance(s2_x, s2_y, Rg_x, Rg_y);
        int a2RgBypass = a2Rg + 2 * (getMahattanDistance(s1_x, s1_y, Rs.first, Rs.second) + 1);


        int a1_extended = (a2k - conflict_k >= 0 ? a2k - conflict_k : 0)/2;
        int a2_extended = a1k/2;

        int R1_x, R1_y, R2_x, R2_y, G1_x, G1_y, G2_x, G2_y, G1_t, G2_t,E1_t,E2_t;

        if ((
                (((s2_x - s1_x) * (s1_x - g1_x) < 0 && (s2_y - s1_y) * (s1_y - g1_y) < 0) && ((s1_x - g1_x) * (g1_x - g2_x) > 0 || (s1_y - g1_y) * (g1_y - g2_y) < 0))
                ||
                (((s1_x - s2_x) * (s2_x - g2_x) < 0 && (s1_y - s2_y) * (s2_y - g2_y) < 0) && ((s2_x - g2_x) * (g2_x - g1_x) < 0 || (s2_y - g2_y) * (g2_y - g1_y) > 0))
        )) { // s1 always in the middle,s2 always between s1 and g1 && g1 lies right side of s1 s2 g2 line
            // or s2 always in the middle, s1 always between s2 and g2 && g2 lies left side of s2 s1 g1 line
            int sign_a1 = g1_y - s1_y >= 0 ? 1 : -1;
            int sign_a2 = g2_x - s2_x >= 0 ? 1 : -1;

            int a1_exit = Rg_y + sign_a1 * a1_extended;
			int a1_entrance = Rs.second - sign_a1 * a1_extended;

//			a1_entrance = sign_a1 * a1_entrance > sign_a1*s1_y ? a1_entrance : s1_y;


            int a2_exit = Rg_x + sign_a2 * a2_extended;
			int a2_entrance = Rs.first - +sign_a2 * a2_extended;
//			a2_entrance = sign_a2 * a2_entrance > sign_a2 * s2_x ? a2_entrance : s2_x;

            if (sign_a1 * a1_entrance < sign_a1 * s1_y || sign_a1 * a1_exit > sign_a1 * g1_y){
                return 2;
            }
            if (sign_a2 * a2_entrance < sign_a2 * s2_x || sign_a2 * a2_exit > sign_a2 * g2_x){
                return 1;
            }


            R1_x = Rs.first;
            G1_x = Rg_x;

            R2_y = Rs.second;
            G2_y = Rg_y;

            G1_t = rt1 + getMahattanDistance(Rs.first, Rs.second, G1_x, a1_exit);
            E1_t = rt1 + getMahattanDistance(Rs.first, a1_entrance, G1_x, a1_entrance);
            G2_t = rt2 + getMahattanDistance(Rs.first, Rs.second, a2_exit,G2_y);
            E2_t = rt2 + getMahattanDistance(a2_entrance, Rs.second, a2_entrance, G2_y);


            std::list<Constraint> constraint11;
            addGeneralKVerticalBarrierConstraint(*paths[a1], a1_exit, R1_x, G1_x, G1_t, num_col, S1_t, constraint11, a1k, a1kMDD);
            multiConstraint1.push_back(constraint11);


            //chasing case always 4 way split
            std::list<Constraint> constraint12;
            addGeneralKVerticalBarrierConstraint(*paths[a1], a1_entrance, R1_x, G1_x, E1_t, num_col, S1_t, constraint12, a1k, a1kMDD);
            multiConstraint1.push_back(constraint12);


            std::list<Constraint> constraint21;
            addGeneralKHorizontalBarrierConstraint(*paths[a2], a2_exit, R2_y, G2_y, G2_t, num_col, S2_t, constraint21, a2k, a2kMDD);
            multiConstraint2.push_back(constraint21);

            //chasing case always 4 way split
            std::list<Constraint> constraint22;
            addGeneralKHorizontalBarrierConstraint(*paths[a2], a2_entrance, R2_y, G2_y, E2_t, num_col, S2_t, constraint22, a2k, a2kMDD);
            multiConstraint2.push_back(constraint22);
            this->isChasing = true;


        }
        else if ((
                (((s1_x - s2_x) * (s2_x - g2_x) < 0 && (s1_y - s2_y) * (s2_y - g2_y) < 0) && ((s2_x - g2_x) * (g2_x - g1_x) > 0 || (s2_y - g2_y) * (g2_y - g1_y) < 0))
                ||
                (((s2_x - s1_x) * (s1_x - g1_x) < 0 && (s2_y - s1_y) * (s1_y - g1_y) < 0) && ((s1_x - g1_x) * (g1_x - g2_x) < 0 || (s1_y - g1_y) * (g1_y - g2_y) > 0))
        ))
        {
            // s2 always in the middle, s1 always between s2 and g2 && g2 lies right side of s2 s1 g1 line
            //or s1 always in the middle,s2 always between s1 and g1 && g1 lies left side of s1 s2 g2 line

            int sign_a1 = g1_x - s1_x >= 0 ? 1 : -1;
            int sign_a2 = g2_y - s2_y >= 0 ? 1 : -1;

            int a1_exit = Rg_x + sign_a1 * a1_extended;
            //a1_exit = sign_a1 * a1_exit < sign_a1*g1_x ? a1_exit : g1_x;
			int a1_entrance = Rs.first - sign_a1 * a1_extended;
//			a1_entrance = sign_a1 * a1_entrance > sign_a1*s1_x ? a1_entrance : s1_x;


            int a2_exit = Rg_y + sign_a2 * a2_extended;
//          a2_exit = sign_a2 * a2_exit < sign_a2*g2_y ? a2_exit : g2_y;
			int a2_entrance = Rs.second - +sign_a2 * a2_extended;
//			a2_entrance = sign_a2 * a2_entrance > sign_a2 * s2_y ? a2_entrance : s2_y;

            if (sign_a1 * a1_entrance < sign_a1 * s1_x || sign_a1 * a1_exit > sign_a1 * g1_x){
                return 2;
            }
            if (sign_a2 * a2_entrance < sign_a2 * s2_y || sign_a2 * a2_exit > sign_a2 * g2_y){
                return 1;
            }

            R2_x = Rs.first;
            G2_x = Rg_x;

            R1_y = Rs.second;
            G1_y = Rg.second;


            G1_t = rt1 + getMahattanDistance(Rs.first, Rs.second, a1_exit, G1_y);
            E1_t = rt1 + getMahattanDistance(a1_entrance, Rs.second, a1_entrance, G1_y);
            G2_t = rt2 + getMahattanDistance(Rs.first, Rs.second, G2_x, a2_exit);
            E2_t = rt2 + getMahattanDistance(Rs.first, a2_entrance, G2_x, a2_entrance);


            std::list<Constraint> constraint11;

            addGeneralKHorizontalBarrierConstraint(*paths[a1], a1_exit, R1_y, G1_y, G1_t, num_col, S1_t, constraint11, a1k, a1kMDD);
            multiConstraint1.push_back(constraint11);

            //chasing case always 4 way split
            std::list<Constraint> constraint12;
            addGeneralKHorizontalBarrierConstraint(*paths[a1], a1_entrance, R1_y, G1_y, E1_t, num_col, S1_t, constraint12, a1k, a1kMDD);
            multiConstraint1.push_back(constraint12);




            std::list<Constraint> constraint21;
            addGeneralKVerticalBarrierConstraint(*paths[a2], a2_exit, R2_x, G2_x, G2_t, num_col, S2_t, constraint21, a2k, a2kMDD);
            multiConstraint2.push_back(constraint21);

            //chasing case always 4 way split
            std::list<Constraint> constraint22;
            addGeneralKVerticalBarrierConstraint(*paths[a2], a2_entrance, R2_x, G2_x, E2_t, num_col, S2_t, constraint22, a2k, a2kMDD);
            multiConstraint2.push_back(constraint22);

            this->isChasing = true;


        }
        else if ((s1_x == s2_x && (s1_y - s2_y) * (s2_y - Rg_y) < 0) ||
                 (s1_x != s2_x && (s1_x - s2_x)*(s2_x - Rg_x) >= 0))
        {


            int sign_a1 = g1_x - s1_x >= 0 ? 1 : -1;
            int sign_a2 = g2_y - s2_y >= 0 ? 1 : -1;

            int a1_exit = Rg_x + sign_a1 * a1_extended;
//            a1_exit = sign_a1 * a1_exit < sign_a1*g1_x ? a1_exit : g1_x;
			int a1_entrance = Rs.first - sign_a1 * a1_extended;
//			a1_entrance = sign_a1 * a1_entrance > sign_a1*s1_x ? a1_entrance : s1_x;


            int a2_exit = Rg_y + sign_a2 * a2_extended;
//            a2_exit = sign_a2 * a2_exit < sign_a2*g2_y ? a2_exit : g2_y;
			int a2_entrance = Rs.second - +sign_a2 * a2_extended;
//			a2_entrance = sign_a2 * a2_entrance > sign_a2 * s2_y ? a2_entrance : s2_y;

            if (sign_a1 * a1_entrance < sign_a1 * s1_x || sign_a1 * a1_exit > sign_a1 * g1_x){
                return 2;
            }
            if (sign_a2 * a2_entrance < sign_a2 * s2_y || sign_a2 * a2_exit > sign_a2 * g2_y){
                return 1;
            }


            R2_x = Rs.first;
            G2_x = Rg_x;

            R1_y = Rs.second;
            G1_y = Rg.second;


            G1_t = rt1 + getMahattanDistance(Rs.first, Rs.second, a1_exit, G1_y);
            E1_t = rt1 + getMahattanDistance(a1_entrance, Rs.second, a1_entrance, G1_y);
            G2_t = rt2 + getMahattanDistance(Rs.first, Rs.second, G2_x, a2_exit);
            E2_t = rt2 + getMahattanDistance(Rs.first, a2_entrance, G2_x, a2_entrance);


            std::list<Constraint> constraint11;

            addGeneralKHorizontalBarrierConstraint(*paths[a1], a1_exit, R1_y, G1_y, G1_t, num_col, S1_t, constraint11, a1k, a1kMDD);
            multiConstraint1.push_back(constraint11);

            if (split4way || a1_4way) {
                std::list<Constraint> constraint12;
                addGeneralKHorizontalBarrierConstraint(*paths[a1], a1_entrance, R1_y, G1_y, E1_t, num_col, S1_t, constraint12, a1k, a1kMDD);
                multiConstraint1.push_back(constraint12);
            }



            std::list<Constraint> constraint21;
            addGeneralKVerticalBarrierConstraint(*paths[a2], a2_exit, R2_x, G2_x, G2_t, num_col, S2_t, constraint21, a2k, a2kMDD);
            multiConstraint2.push_back(constraint21);

            if (split4way || a2_4way) {
                std::list<Constraint> constraint22;
                addGeneralKVerticalBarrierConstraint(*paths[a2], a2_entrance, R2_x, G2_x, E2_t, num_col, S2_t, constraint22, a2k, a2kMDD);
                multiConstraint2.push_back(constraint22);
            }


        }
        else
        {

            int sign_a1 = g1_y - s1_y >= 0 ? 1 : -1;
            int sign_a2 = g2_x - s2_x >= 0 ? 1 : -1;

            int a1_exit = Rg_y + sign_a1 * a1_extended;
//            a1_exit = sign_a1 * a1_exit < sign_a1*g1_y ? a1_exit : g1_y;
			int a1_entrance = Rs.second - sign_a1 * a1_extended;
//			a1_entrance = sign_a1 * a1_entrance > sign_a1*s1_y ? a1_entrance : s1_y;


            int a2_exit = Rg_x + sign_a2 * a2_extended;
//            a2_exit = sign_a2 * a2_exit < sign_a2*g2_x ? a2_exit : g2_x;
			int a2_entrance = Rs.first - +sign_a2 * a2_extended;
//			a2_entrance = sign_a2 * a2_entrance > sign_a2 * s2_x ? a2_entrance : s2_x;

            if (sign_a1 * a1_entrance < sign_a1 * s1_y || sign_a1 * a1_exit > sign_a1 * g1_y){
                return 2;
            }
            if (sign_a2 * a2_entrance < sign_a2 * s2_x || sign_a2 * a2_exit > sign_a2 * g2_x){
                return 1;
            }

            R1_x = Rs.first;
            G1_x = Rg_x;

            R2_y = Rs.second;
            G2_y = Rg_y;

            G1_t = rt1 + getMahattanDistance(Rs.first, Rs.second, G1_x, a1_exit);
            E1_t = rt1 + getMahattanDistance(Rs.first, a1_entrance, G1_x, a1_entrance);
            G2_t = rt2 + getMahattanDistance(Rs.first, Rs.second, a2_exit, G2_y);
            E2_t = rt2 + getMahattanDistance(a2_entrance, Rs.second, a2_entrance, G2_y);


            std::list<Constraint> constraint11;
            addGeneralKVerticalBarrierConstraint(*paths[a1], a1_exit, R1_x, G1_x, G1_t, num_col, S1_t, constraint11, a1k, a1kMDD);
            multiConstraint1.push_back(constraint11);


            if (split4way || a1_4way) {
                std::list<Constraint> constraint12;
                addGeneralKVerticalBarrierConstraint(*paths[a1], a1_entrance, R1_x, G1_x, E1_t, num_col, S1_t, constraint12, a1k, a1kMDD);
                multiConstraint1.push_back(constraint12);
            }


            std::list<Constraint> constraint21;
            addGeneralKHorizontalBarrierConstraint(*paths[a2], a2_exit, R2_y, G2_y, G2_t, num_col, S2_t, constraint21, a2k, a2kMDD);
            multiConstraint2.push_back(constraint21);

            if (split4way || a2_4way) {
                std::list<Constraint> constraint22;
                addGeneralKHorizontalBarrierConstraint(*paths[a2], a2_entrance, R2_y, G2_y, E2_t, num_col, S2_t, constraint22, a2k, a2kMDD);
                multiConstraint2.push_back(constraint22);
            }

        }
        type = conflict_type::RECTANGLE4;
        return 0;
    }
//	bool flippedRectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
//		const std::pair<int, int>& s1, const std::pair<int, int>& s2, int Rg_t,
//		const std::vector<Path*>& paths, int S1_t, int S2_t, const std::pair<int, int>& G1, const std::pair<int, int>& G2,
//		int num_col, int k,int flipType, MDDLevels* a1kMDD = NULL, MDDLevels* a2kMDD = NULL) // For K-RM
//	{
//
//		this->a1 = a1;
//		this->a2 = a2;
//		this->t_sg = Rg_t - abs(Rg.first - Rs.first) - abs(Rg.second - Rs.second);
//		this->rs = Rs.first*num_col + Rs.second;
//		this->rg = Rg.first*num_col + Rg.second;
//		this->flipType = flipType;
//
//		int s1_x = s1.first;
//		int s1_y = s1.second;
//		int s2_x = s2.first;
//		int s2_y = s2.second;
//		int Rg_x = Rg.first;
//		int Rg_y = Rg.second;
//		int g1_x = G1.first;
//		int g1_y = G1.second;
//		int g2_x = G2.first;
//		int g2_y = G2.second;
//
//
//		int R1_x, R1_y, R2_x, R2_y, G1_x, G1_y, G2_x, G2_y;
//		vector<int> horizontal;
//		vector<int> horizontalMin;
//		vector<int> horizontalMax;
//
//		vector<int> vertical;
//		vector<int> verticalMin;
//		vector<int> verticalMax;
//
//		if (flipType == 2) {
//			if ((g2_x == s1_x && (g1_y - s1_y) * (s1_y - Rg_y) >= 0) ||
//				(g2_x != s1_x && (g2_x - s1_x) * (s1_x - Rg_x) < 0)) {
//
//				G1_x = Rg.first;
//				G1_y = g2_y;
//				R1_y = s2_y;
//
//				int sign = R1_y < G1_y ? 1 : -1;
//				horizontal.clear();
//				horizontalMin.clear();
//				horizontalMin.clear();
//				for (int y = G1_y; y != R1_y - sign; y = y - sign * 1) {
//					horizontal.push_back(y);
//					horizontalMin.push_back(getMahattanDistance(s1_x, s1_y, G1_x, y) + S1_t);
//					horizontalMax.push_back(getMahattanDistance(s2_x, s2_y, G1_x, y) + k + S2_t);
//				}
//
//				if (!addFlippedHorizontalLongBarrierConstraint(*paths[a1], G1_x, horizontal, horizontalMin, horizontalMax, num_col, S1_t, constraint1, k, a1kMDD))
//					return false;
//
//				G2_y = Rs.second;
//				G2_x = g1_x;
//				R2_x = s1_x;
//
//				sign = R2_x < G2_x ? 1 : -1;
//				vertical.clear();
//				verticalMin.clear();
//				verticalMax.clear();
//				for (int x = G2_x; x != R2_x - sign; x = x - sign * 1) {
//					vertical.push_back(x);
//					verticalMin.push_back(getMahattanDistance(s2_x, s2_y, x, G2_y) + S2_t);
//					verticalMax.push_back(getMahattanDistance(s1_x, s1_y, x, G2_y) + k + S1_t);
//				}
//
//				if (!addFlippedVerticalLongBarrierConstraint(*paths[a2], G2_y, vertical, verticalMin, verticalMax, num_col, S2_t, constraint2, k, a2kMDD))
//					return false;
//
//
//			}
//			else {
//				G1_y = Rg.second;
//				G1_x = g2_x;
//				R1_x = s2_x;
//
//				int sign = R1_x < G1_x ? 1 : -1;
//				vertical.clear();
//				verticalMin.clear();
//				verticalMax.clear();
//				for (int x = G1_x; x != R1_x - sign; x = x - sign * 1) {
//					vertical.push_back(x);
//					verticalMin.push_back(getMahattanDistance(s1_x, s1_y, x, G1_y) + S1_t);
//					verticalMax.push_back(getMahattanDistance(s2_x, s2_y, x, G1_y) + k + S2_t);
//				}
//
//				if (!addFlippedVerticalLongBarrierConstraint(*paths[a1], G1_y, vertical, verticalMin, verticalMax, num_col, S1_t, constraint1, k, a1kMDD))
//					return false;
//
//				G2_x = Rs.first;
//				G2_y = g1_y;
//				R2_y = s1_y;
//
//				sign = R2_y < G2_y ? 1 : -1;
//				horizontal.clear();
//				horizontalMin.clear();
//				horizontalMin.clear();
//				for (int y = G2_y; y != R2_y - sign; y = y - sign * 1) {
//					horizontal.push_back(y);
//					horizontalMin.push_back(getMahattanDistance(s2_x, s2_y, G2_x, y) + S2_t);
//					horizontalMax.push_back(getMahattanDistance(s1_x, s1_y, G2_x, y) + k + S1_t);
//				}
//
//				if (!addFlippedHorizontalLongBarrierConstraint(*paths[a2], G2_x, horizontal, horizontalMin, horizontalMax, num_col, S2_t, constraint2, k, a2kMDD))
//					return false;
//
//			}
//
//
//
//
//
//		}
//		else if (flipType == 1) {//1 flipped case
//			if ((s1_y - g1_y) * (s2_y - g2_y) < 0) {// y dimension flipped
//				if (Rs.second != g1_y) {//s1 vertical border need constraint
//					G1_y = Rs.second;
//					G1_x = g2_x;
//					R1_x = s2_x;
//
//					int sign = R1_x < G1_x ? 1 : -1;
//					vertical.clear();
//					verticalMin.clear();
//					verticalMax.clear();
//					for (int x = G1_x; x != R1_x-sign; x = x - sign * 1) {
//						vertical.push_back(x);
//						//int tMin = min(getMahattanDistance(s1_x, s1_y, x, G1_y) + S1_t, getMahattanDistance(s2_x, s2_y, x, G1_y) + S2_t);
//						//verticalMin.push_back(tMin);
//						//verticalMax.push_back(tMin+k);
//						verticalMin.push_back(getMahattanDistance(s1_x, s1_y, x, G1_y) + S1_t);
//						verticalMax.push_back(getMahattanDistance(s2_x, s2_y, x, G1_y) + k + S2_t);
//					}
//
//					if (!addFlippedVerticalLongBarrierConstraint(*paths[a1], G1_y, vertical, verticalMin, verticalMax, num_col, S1_t, constraint1, k, a1kMDD))
//						return false;
//
//				}
//
//				if (Rg.first != g1_x || (Rs.second == g1_y && Rg.first == g1_x)) {//s1 horizontoal border need constraint
//					G1_x = Rg.first;
//					G1_y = g2_y;
//					R1_y = s2_y;
//
//					int sign = R1_y < G1_y ? 1 : -1;
//					horizontal.clear();
//					horizontalMin.clear();
//					horizontalMin.clear();
//					for (int y = G1_y; y != R1_y-sign; y = y - sign * 1) {
//						horizontal.push_back(y);
//						//int tMin = min(getMahattanDistance(s1_x, s1_y, G1_x, y) + S1_t, getMahattanDistance(s2_x, s2_y, G1_x, y) + S2_t);
//						//horizontalMin.push_back(tMin);
//						//horizontalMax.push_back(tMin + k);
//						horizontalMin.push_back(getMahattanDistance(s1_x, s1_y, G1_x, y) + S1_t);
//						horizontalMax.push_back(getMahattanDistance(s2_x, s2_y, G1_x, y) + k + S2_t);
//					}
//
//					if (!addFlippedHorizontalLongBarrierConstraint(*paths[a1], G1_x, horizontal, horizontalMin, horizontalMax, num_col, S1_t, constraint1, k, a1kMDD))
//						return false;
//
//				}
//
//				if (Rg.second != g2_y ) {//s2 vertical border need constraint
//					G2_y = Rg.second;
//					G2_x = g1_x;
//					R2_x = s1_x;
//
//					int sign = R2_x < G2_x ? 1 : -1;
//					vertical.clear();
//					verticalMin.clear();
//					verticalMax.clear();
//					for (int x = G2_x; x != R2_x - sign; x = x - sign * 1) {
//						vertical.push_back(x);
//						/*int tMin = min(getMahattanDistance(s1_x, s1_y, x, G2_y) + S1_t, getMahattanDistance(s2_x, s2_y, x, G2_y) + S2_t);
//						verticalMin.push_back(tMin);
//						verticalMax.push_back(tMin + k);*/
//						verticalMin.push_back(getMahattanDistance(s2_x, s2_y, x, G2_y) + S2_t);
//						verticalMax.push_back(getMahattanDistance(s1_x, s1_y, x, G2_y) + k + S1_t);
//					}
//
//					if (!addFlippedVerticalLongBarrierConstraint(*paths[a2], G2_y, vertical, verticalMin, verticalMax, num_col, S2_t, constraint2, k, a2kMDD))
//						return false;
//
//				}
//
//				if (Rg.first != g2_x || (Rg.second == g2_y && Rg.first == g2_x)) {//s2 horizontoal border need constraint
//					G2_x = Rg.first;
//					G2_y = g1_y;
//					R2_y = s1_y;
//
//					int sign = R2_y < G2_y ? 1 : -1;
//					horizontal.clear();
//					horizontalMin.clear();
//					horizontalMin.clear();
//					for (int y = G2_y; y != R2_y - sign; y = y - sign * 1) {
//						horizontal.push_back(y);
//						/*int tMin = min(getMahattanDistance(s1_x, s1_y, G2_x, y) + S1_t, getMahattanDistance(s2_x, s2_y, G2_x, y) + S2_t);
//						horizontalMin.push_back(tMin);
//						horizontalMax.push_back(tMin + k);*/
//						horizontalMin.push_back(getMahattanDistance(s2_x, s2_y, G2_x, y) + S2_t);
//						horizontalMax.push_back(getMahattanDistance(s1_x, s1_y, G2_x, y) + k + S1_t);
//					}
//
//					if (!addFlippedHorizontalLongBarrierConstraint(*paths[a2], G2_x, horizontal, horizontalMin, horizontalMax, num_col, S2_t, constraint2, k, a2kMDD))
//						return false;
//
//				}
//
//
//			}
//			else {// x dimension flipped
//				if (Rg.second != g1_y || (Rg.second == g1_y && Rs.first == g1_x)) {//s1 vertical border need constraint
//					G1_y = Rg.second;
//					G1_x = g2_x;
//					R1_x = s2_x;
//
//					int sign = R1_x < G1_x ? 1 : -1;
//					vertical.clear();
//					verticalMin.clear();
//					verticalMax.clear();
//					for (int x = G1_x; x != R1_x-sign; x = x - sign * 1) {
//						vertical.push_back(x);
//						/*int tMin = min(getMahattanDistance(s1_x, s1_y, x, G1_y) + S1_t, getMahattanDistance(s2_x, s2_y, x, G1_y) + S2_t);
//						verticalMin.push_back(tMin);
//						verticalMax.push_back(tMin + k);*/
//						verticalMin.push_back(getMahattanDistance(s1_x, s1_y, x, G1_y) + S1_t);
//						verticalMax.push_back(getMahattanDistance(s2_x, s2_y, x, G1_y) + k + S2_t);
//					}
//
//					if (!addFlippedVerticalLongBarrierConstraint(*paths[a1], G1_y, vertical, verticalMin, verticalMax, num_col, S1_t, constraint1, k, a1kMDD))
//						return false;
//
//				}
//
//				if (Rs.first != g1_x) {//s1 horizontoal border need constraint
//					G1_x = Rs.first;
//					G1_y = g2_y;
//					R1_y = s2_y;
//
//					int sign = R1_y < G1_y ? 1 : -1;
//					horizontal.clear();
//					horizontalMin.clear();
//					horizontalMin.clear();
//					for (int y = G1_y; y != R1_y-sign; y = y - sign * 1) {
//						horizontal.push_back(y);
//						/*int tMin = min(getMahattanDistance(s1_x, s1_y, G1_x, y) + S1_t, getMahattanDistance(s2_x, s2_y, G1_x, y) + S2_t);
//						horizontalMin.push_back(tMin);
//						horizontalMax.push_back(tMin + k);*/
//						horizontalMin.push_back(getMahattanDistance(s1_x, s1_y, G1_x, y) + S1_t);
//						horizontalMax.push_back(getMahattanDistance(s2_x, s2_y, G1_x, y) + k + S2_t);
//					}
//
//					if (!addFlippedHorizontalLongBarrierConstraint(*paths[a1], G1_x, horizontal, horizontalMin, horizontalMax, num_col, S1_t, constraint1, k, a1kMDD))
//						return false;
//
//				}
//
//				if (Rg.second != g2_y || (Rg.second == g2_y && Rg.first == g2_x)) {//s2 vertical border need constraint
//					G2_y = Rg.second;
//					G2_x = g1_x;
//					R2_x = s1_x;
//
//					int sign = R2_x < G2_x ? 1 : -1;
//					vertical.clear();
//					verticalMin.clear();
//					verticalMax.clear();
//					for (int x = G2_x; x != R2_x - sign; x = x - sign * 1) {
//						vertical.push_back(x);
//						/*int tMin = min(getMahattanDistance(s1_x, s1_y, x, G2_y) + S1_t, getMahattanDistance(s2_x, s2_y, x, G2_y) + S2_t);
//						verticalMin.push_back(tMin);
//						verticalMax.push_back(tMin + k);*/
//						verticalMin.push_back(getMahattanDistance(s2_x, s2_y, x, G2_y) + S2_t);
//						verticalMax.push_back(getMahattanDistance(s1_x, s1_y, x, G2_y) + k + S1_t);
//					}
//
//					if (!addFlippedVerticalLongBarrierConstraint(*paths[a2], G2_y, vertical, verticalMin, verticalMax, num_col, S2_t, constraint2, k, a2kMDD))
//						return false;
//
//				}
//
//				if (Rg.first != g2_x) {//s2 horizontoal border need constraint
//					G2_x = Rg.first;
//					G2_y = g1_y;
//					R2_y = s1_y;
//
//					int sign = R2_y < G2_y ? 1 : -1;
//					horizontal.clear();
//					horizontalMin.clear();
//					horizontalMin.clear();
//					for (int y = G2_y; y != R2_y - sign; y = y - sign * 1) {
//						horizontal.push_back(y);
//						/*int tMin = min(getMahattanDistance(s1_x, s1_y, G2_x, y) + S1_t, getMahattanDistance(s2_x, s2_y, G2_x, y) + S2_t);
//						horizontalMin.push_back(tMin);
//						horizontalMax.push_back(tMin + k);*/
//						horizontalMin.push_back(getMahattanDistance(s2_x, s2_y, G2_x, y) + S2_t);
//						horizontalMax.push_back(getMahattanDistance(s1_x, s1_y, G2_x, y) + k + S1_t);
//					}
//
//					if (!addFlippedHorizontalLongBarrierConstraint(*paths[a2], G2_x, horizontal, horizontalMin, horizontalMax, num_col, S2_t, constraint2, k, a2kMDD))
//						return false;
//
//				}
//			}
//
//
//
//		}
//		else {//standard non flip
//
//			if ((s1_x == s2_x && (s1_y - s2_y) * (s2_y - Rg_y) < 0) ||
//				(s1_x != s2_x && (s1_x - s2_x)*(s2_x - Rg_x) >= 0))
//			{
//				R1_x = Rg_x;
//				G1_x = Rg_x;
//				R1_y = s2_y;
//				G1_y = g2_y;
//
//				R2_y = Rg_y;
//				G2_y = Rg_y;
//				R2_x = s1_x;
//				G2_x = g1_x;
//
//
//				//cout << "s1t" << S1_t << "G1_t" << G1_t << "G1_y " << G1_y << endl;
//				//cout << "s2t" << S2_t << "G2_t" << G2_t << "G2_x " << G2_x << endl;
//
//				int sign = R1_y < G1_y ? 1 : -1;
//				for (int y = G1_y; y != R1_y-sign; y = y - sign * 1) {
//					horizontal.push_back(y);
//					horizontalMin.push_back(getMahattanDistance(s1_x, s1_y, Rg_x, y) + S1_t);
//					horizontalMax.push_back(getMahattanDistance(s2_x, s2_y, Rg_x, y) + k + S2_t);
//				}
//
//
//				sign = R2_x < G2_x ? 1 : -1;
//				for (int x = G2_x; x != R2_x-sign; x = x - sign * 1) {
//					vertical.push_back(x);
//					verticalMin.push_back(getMahattanDistance(s2_x, s2_y, x, Rg_y) + S2_t);
//					verticalMax.push_back(getMahattanDistance(s1_x, s1_y, x, Rg_y) + k + S1_t);
//				}
//
//				//cout << "Horizontal length: " << horizontal.size() << " Vertical length: " << vertical.size() << endl;
//
//
//				if (!addFlippedHorizontalLongBarrierConstraint(*paths[a1], Rg_x, horizontal, horizontalMin, horizontalMax, num_col, S1_t, constraint1, k, a1kMDD))
//					return false;
//				if (!addFlippedVerticalLongBarrierConstraint(*paths[a2], Rg_y, vertical, verticalMin, verticalMax, num_col, S2_t, constraint2, k, a2kMDD))
//					return false;
//			}
//			else
//			{
//				R1_y = Rg_y;
//				G1_y = Rg_y;
//				R1_x = s2_x;
//				G1_x = g2_x;
//
//
//				R2_x = Rg_x;
//				G2_x = Rg_x;
//				R2_y = s1_y;
//				G2_y = g1_y;
//
//
//
//
//
//				int sign = R2_y < G2_y ? 1 : -1;
//
//				for (int y = G2_y; y != R2_y-sign; y = y - sign * 1) {
//					horizontal.push_back(y);
//					horizontalMin.push_back(getMahattanDistance(s2_x, s2_y, Rg_x, y) + S2_t);
//					horizontalMax.push_back(getMahattanDistance(s1_x, s1_y, Rg_x, y) + k + S1_t);
//				}
//
//				sign = R1_x < G1_x ? 1 : -1;
//				for (int x = G1_x; x != R1_x-sign; x = x - sign * 1) {
//					vertical.push_back(x);
//					verticalMin.push_back(getMahattanDistance(s1_x, s1_y, x, Rg_y)+S1_t);
//					verticalMax.push_back(getMahattanDistance(s2_x, s2_y, x, Rg_y) + k + S2_t);
//				}
//
//
//				if (!addFlippedVerticalLongBarrierConstraint(*paths[a1], Rg_y, vertical, verticalMin, verticalMax, num_col, S1_t, constraint1, k, a1kMDD))
//					return false;
//				if (!addFlippedHorizontalLongBarrierConstraint(*paths[a2], Rg_x, horizontal, horizontalMin, horizontalMax, num_col, S2_t, constraint2, k, a2kMDD))
//					return false;
//				//exit(0);
//			}
//		}
//
//		type = conflict_type::RECTANGLE;
//		return true;
//	}

	void targetConflict(int a1, int a2, int v, int t,int kDelay)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->k = 0;
		this->originalConf1 = -1;
		this->originalConf2 = a1;

		this->constraint1.emplace_back(-1, a1, t + kDelay/* kDelay>0? t + kDelay+1:t*/, constraint_type::LENGTH); // length of a1 should be larger than t
		this->constraint2.emplace_back(v, a1, t, constraint_type::LENGTH); // length of a1 should be no larger than t, and other agents can not use v at and after timestep t
		type = conflict_type::TARGET;
	}


};

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);

bool operator < (const Conflict& conflict1, const Conflict& conflict2);





