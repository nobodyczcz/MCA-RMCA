#include "Conflict.h"

std::ostream& operator<<(std::ostream& os, const Constraint& constraint)
{
	os << "<" << std::get<0>(constraint) << "," << std::get<1>(constraint) << "," <<
		std::get<2>(constraint) << "," << std::get<3>(constraint) << ">";
	return os;
}


std::ostream& operator<<(std::ostream& os, const Conflict& conflict)
{
	switch (conflict.p)
	{
		case conflict_priority::CARDINAL:
			os << "cardinal ";
			break;
		case conflict_priority::SEMI:
			os << "semi-cardinal ";
			break;
		case conflict_priority::NON:
			os << "non-cardinal ";
			break;
	}

	switch (conflict.type)
	{
		case conflict_type::STANDARD:
			os << "standard";
			break;
		case conflict_type::RECTANGLE:
			os <<conflict.flipType << " flip rectangle";
			break;
		case conflict_type::RECTANGLE4:
			os << conflict.flipType << " flip rectangle";
			break;
		case conflict_type::CORRIDOR2:
			os << "corrdior2";
			break;
		case conflict_type::CORRIDOR4:
			os << "corrdior4";
			break;
		case conflict_type::TARGET:
			os << "target";
	}

	os << " conflict:  " << conflict.a1 << " with ";
	for (auto con : conflict.constraint1)
		os << con << ",";	
	for (auto cons : conflict.multiConstraint1) {
		os << " node: ";
		for (auto con1 : cons)
			os << con1 << ",";
	}


	os << " and " << conflict.a2 << " with ";
	for (auto con : conflict.constraint2)
		os << con << ",";	

	for (auto cons : conflict.multiConstraint2) {
		os << " node: ";
		for (auto con1 : cons)
			os << con1 << ",";
	}
	os << std::endl;
	return os;
}

bool operator < (const Conflict& conflict1, const Conflict& conflict2) // return true if conflict2 has higher priority
{
	if (conflict1.type == conflict_type::TARGET && conflict2.type == conflict_type::TARGET)
	{
		if (conflict1.p < conflict2.p)
			return false;
		else
			return true;
	}
	else if (conflict1.type == conflict_type::TARGET)
		return false;
	else if (conflict2.type == conflict_type::TARGET)
		return true;
	

	if (conflict1.p < conflict2.p)
		return false;
	else if (conflict1.p > conflict2.p)
		return true;
	else if (conflict1.p == conflict_priority::CARDINAL) // both are cardinal
	{
	    if(conflict1.type == conflict_type::RECTANGLE||conflict1.type == conflict_type::RECTANGLE4){
            if (conflict2.type != conflict_type::RECTANGLE ||conflict2.type != conflict_type::RECTANGLE4)
                return false;
	    }
	    else if(conflict2.type == conflict_type::RECTANGLE||conflict2.type == conflict_type::RECTANGLE4){
                return true;
        }
		else if (conflict1.type == conflict_type::CORRIDOR2)
		{
			if (conflict2.type != conflict_type::CORRIDOR2)
				return false;
		}
		else if (conflict1.type == conflict_type::CORRIDOR4)
		{
			if (conflict2.type == conflict_type::CORRIDOR2)
			{
				return true;
			}
			else if (conflict2.type != conflict_type::CORRIDOR4)
				return false;
		}
		else if (conflict2.type == conflict_type::CORRIDOR4 || conflict2.type == conflict_type::CORRIDOR2)
		{
			return true;
		}
	}
	else // both are semi or both are non 
	{
        if(conflict1.type == conflict_type::RECTANGLE||conflict1.type == conflict_type::RECTANGLE4){
            if (conflict2.type != conflict_type::RECTANGLE ||conflict2.type != conflict_type::RECTANGLE4)
                return false;
        }
        else if(conflict2.type == conflict_type::RECTANGLE||conflict2.type == conflict_type::RECTANGLE4){
            return true;
        }
		else if (conflict2.type == conflict_type::CORRIDOR2 &&  conflict1.type != conflict_type::CORRIDOR2)
		{
			return true;
		}
		else	if (conflict2.type != conflict_type::CORRIDOR2 &&  conflict1.type == conflict_type::CORRIDOR2)
		{
			return false;
		}
		
		/*if (conflict2.type == conflict_type::RECTANGLE &&  conflict1.type != conflict_type::RECTANGLE)
		{
			return true;
		}*/
	}



	if (conflict2.t < conflict1.t)
	{
		return true;
	}
	else
		return false;
}


// add a pair of barrier constraints
void addBarrierConstraints(int S1, int S2, int S1_t, int S2_t, int Rg, int num_col,
	std::list<Constraint>& constraints1, std::list<Constraint>& constraints2)
{
	int s1_x = S1 / num_col;
	int s1_y = S1 % num_col;
	int s2_x = S2 / num_col;
	int s2_y = S2 % num_col;
	int Rg_x = Rg / num_col;
	int Rg_y = Rg % num_col;
	int Rg_t = S1_t + abs(Rg_x - s1_x) + abs(Rg_y - s1_y);

	int R1_x, R1_y, R2_x, R2_y;
	if (s1_x == s2_x)
	{
		if ((s1_y - s2_y) * (s2_y - Rg_y) >= 0)
		{
			R1_x = s1_x;
			R2_x = Rg_x;
			R1_y = Rg_y;
			R2_y = s2_y;
		}
		else
		{
			R1_x = Rg_x;
			R2_x = s2_x;
			R1_y = s1_y;
			R2_y = Rg_y;
		}
	}
	else if ((s1_x - s2_x)*(s2_x - Rg_x) >= 0)
	{
		R1_x = Rg_x;
		R2_x = s2_x;
		R1_y = s1_y;
		R2_y = Rg_y;
	}
	else
	{
		R1_x = s1_x;
		R2_x = Rg_x;
		R1_y = Rg_y;
		R2_y = s2_y;
	}

	constraints1.emplace_back(R1_x * num_col + R1_y, Rg, Rg_t, constraint_type::BARRIER);
	constraints2.emplace_back(R2_x * num_col + R2_y, Rg, Rg_t, constraint_type::BARRIER);
}

// add a pair of modified barrier constraints
bool addModifiedBarrierConstraints(const std::vector<PathEntry>& path1, const std::vector<PathEntry>& path2,
	int S1_t, int S2_t, int Rg, int num_col,
	std::list<Constraint>& constraints1, std::list<Constraint>& constraints2)
{
	int s1_x = path1[S1_t].location / num_col;
	int s1_y = path1[S1_t].location % num_col;
	int s2_x = path2[S2_t].location / num_col;
	int s2_y = path2[S2_t].location % num_col;
	int Rg_x = Rg / num_col;
	int Rg_y = Rg % num_col;
	int Rg_t = S1_t + abs(Rg_x - s1_x) + abs(Rg_y - s1_y);

	bool succ1, succ2;
	int R1_x, R1_y, R2_x, R2_y;
	if ((s1_x == s2_x && (s1_y - s2_y) * (s2_y - Rg_y) < 0) ||
		(s1_x != s2_x && (s1_x - s2_x)*(s2_x - Rg_x) >= 0))
	{
		R1_x = Rg_x;
		R2_x = s2_x;
		R1_y = s1_y;
		R2_y = Rg_y;
		succ1 = addModifiedHorizontalBarrierConstraint(path1, Rg_x, R1_y, Rg_y, Rg_t, num_col, constraints1);
		succ2 = addModifiedVerticalBarrierConstraint(path2, Rg_y, R2_x, Rg_x, Rg_t, num_col, constraints2);
	}
	else
	{
		R1_x = s1_x;
		R2_x = Rg_x;
		R1_y = Rg_y;
		R2_y = s2_y;
		succ1 = addModifiedVerticalBarrierConstraint(path1, Rg_y, R1_x, Rg_x, Rg_t, num_col, constraints1);
		succ2 = addModifiedHorizontalBarrierConstraint(path2, Rg_x, R2_y, Rg_y, Rg_t, num_col, constraints2);
	}
	return succ1 && succ2;
}


// add a horizontal modified barrier constraint
bool addModifiedHorizontalBarrierConstraint(const std::vector<PathEntry>& path, int x,
	int Ri_y, int Rg_y, int Rg_t, int num_col,
	std::list<Constraint>& constraints)
{
	int sign = Ri_y < Rg_y ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_y - Rg_y);
	int t1 = -1;
	int t_min = std::max(Ri_t, 0);
	int t_max = std::min(Rg_t, (int)path.size() - 1);
	for (int t2 = t_min; t2 <= t_max; t2++)
	{
		int loc = (Ri_y + (t2 - Ri_t) * sign) + x * num_col;
		std::list<int>::const_iterator it = std::find(path[t2].locations.begin(), path[t2].locations.end(), loc);
		if (it == path[t2].locations.end() && t1 >= 0) // add constraints [t1, t2)
		{
			int loc1 = (Ri_y + (t1 - Ri_t) * sign) + x * num_col;
			int loc2 = (Ri_y + (t2 - 1 - Ri_t) * sign) + x * num_col;
			constraints.emplace_back(loc1, loc2, t2 - 1, constraint_type::BARRIER);
			t1 = -1;
			continue;
		}
		else if (it != path[t2].locations.end() && t1 < 0)
		{
			t1 = t2;
		}
		if (it != path[t2].locations.end() && t2 == t_max)
		{
			int loc1 = (Ri_y + (t1 - Ri_t) * sign) + x * num_col;
			constraints.emplace_back(loc1, loc, t2, constraint_type::BARRIER); // add constraints [t1, t2]
		}
	}
	if (constraints.empty())
	{
		// std::cout << "Fail to add modified barrier constraints!" << std::endl;
		return false;
	}
	else
		return true;
}

// add a pair of long k-delay barrier constraints
bool addKDelayBarrierConstraints(int S1, int S2, int S1_t, int S2_t, pair<int, int> Rs, pair<int, int> Rg, int G1, int G2, int num_col,
	std::vector<std::list<Constraint>>& multiConstraint1, std::vector<std::list<Constraint>>& multiConstraint2 , int k, int RM4way)
{
	// 
	int s1_x = S1 / num_col;
	int s1_y = S1 % num_col;
	int s2_x = S2 / num_col;
	int s2_y = S2 % num_col;
	int g1_x = G1 / num_col;
	int g1_y = G1 % num_col;
	int g2_x = G2 / num_col;
	int g2_y = G2 % num_col;
	int Rg_x = Rg.first;
	int Rg_y = Rg.second;
	int Rg_t = S1_t + abs(Rg_x - s1_x) + abs(Rg_y - s1_y);

	int a1Rg = getMahattanDistance(s1_x, s1_y, Rg_x, Rg_y);
	int a1RgBypass = a1Rg + 2 * (getMahattanDistance(s2_x, s2_y, Rs.first, Rs.second) + 1);

	int a2Rg = getMahattanDistance(s2_x, s2_y, Rg_x, Rg_y);
	int a2RgBypass = a2Rg + 2 * (getMahattanDistance(s1_x, s1_y, Rs.first, Rs.second) + 1);

	if (RM4way == 0 && (a2RgBypass <= a1Rg + k || a1RgBypass <= a1Rg + k)) {
		return false;
	}

	int R1_x, R1_y, R2_x, R2_y;
	int Rg1_x, Rg1_y, Rg2_x, Rg2_y;
	int R1e_x, R1e_y, R2e_x, R2e_y;
	int Rg1e_x, Rg1e_y, Rg2e_x, Rg2e_y;
	if (s1_x == s2_x)
	{
		if ((s1_y - s2_y) * (s2_y - Rg_y) >= 0)
		{
			R1_x = s2_x;//different
			Rg1_x = g2_x;
			R2_x = Rg_x;
			Rg2_x = Rg_x;
			R1_y = Rg_y;
			Rg1_y = Rg_y;
			R2_y = s1_y;//different
			Rg2_y = g1_y;

			R1e_x = s2_x;//different
			Rg1e_x = g2_x;
			R2e_x = Rs.first;
			Rg2e_x = Rs.first;
			R1e_y = Rs.second;
			Rg1e_y = Rs.second;
			R2e_y = s1_y;//different
			Rg2e_y = g1_y;
		}
		else
		{
			R1_x = Rg_x;
			Rg1_x = Rg_x;
			R2_x = s1_x;//different
			Rg2_x = g1_x;
			R1_y = s2_y;//different
			Rg1_y = g2_y;
			R2_y = Rg_y;
			Rg2_y = Rg_y;

			R1e_x = Rs.first;
			Rg1e_x = Rs.first;
			R2e_x = s1_x;//different
			Rg2e_x = g1_x;
			R1e_y = s2_y;//different
			Rg1e_y = g2_y;
			R2e_y = Rs.second;
			Rg2e_y = Rs.second;
		}
	}
	else if ((s1_x - s2_x)*(s2_x - Rg_x) >= 0)
	{
		R1_x = Rg_x;
		Rg1_x = Rg_x;
		R2_x = s1_x;//different
		Rg2_x = g1_x;
		R1_y = s2_y;//different
		Rg1_y = g2_y;
		R2_y = Rg_y;
		Rg2_y = Rg_y;

		R1e_x = Rs.first;
		Rg1e_x = Rs.first;
		R2e_x = s1_x;//different
		Rg2e_x = g1_x;
		R1e_y = s2_y;//different
		Rg1e_y = g2_y;
		R2e_y = Rg.second;
		Rg2e_y = Rg.second;
	}
	else
	{
		R1_x = s2_x;//different
		Rg1_x = g2_x;
		R2_x = Rg_x;
		Rg2_x = Rg_x;
		R1_y = Rg_y;
		Rg1_y = Rg_y;
		R2_y = s1_y;//different
		Rg2_y = g1_y;

		R1e_x = s2_x;//different
		Rg1e_x = g2_x;
		R2e_x = Rs.first;
		Rg2e_x = Rs.first;
		R1e_y = Rg.second;
		Rg1e_y = Rg.second;
		R2e_y = s1_y;//different
		Rg2e_y = g1_y;
	}

	int Rg1_t = S1_t + abs(Rg1_x - s1_x) + abs(Rg1_y - s1_y);
	int Rg2_t = S1_t + abs(Rg2_x - s1_x) + abs(Rg2_y - s1_y);




	std::list<Constraint> constraint11;

	for (int i = 0; i <= k; i++) {
		constraint11.emplace_back(R1_x * num_col + R1_y, Rg1_x * num_col + Rg1_y, Rg1_t + i, constraint_type::BARRIER);
	}
	
	multiConstraint1.push_back(constraint11);

	if (RM4way >= 2 || a1RgBypass <= a1Rg + k) {
		std::list<Constraint> constraint12;
		int Rg1e_t = getMahattanDistance(Rg1e_x, Rg1e_y, s1_x, s1_y);

		for (int i = 0; i <= k; i++) {

			constraint12.emplace_back(R1e_x * num_col + R1e_y, Rg1e_x * num_col + Rg1e_y, Rg1e_t + i, constraint_type::BARRIER);
		}
		multiConstraint1.push_back(constraint12);
	}



	std::list<Constraint> constraint21;
	for (int i = 0; i <= k; i++) {
		constraint21.emplace_back(R2_x * num_col + R2_y, Rg2_x * num_col + Rg2_y, Rg2_t + i, constraint_type::BARRIER);
	}

	multiConstraint2.push_back(constraint21);

	if (RM4way >= 2 || a2RgBypass <= a1Rg + k) {// the lower bound is always a1Rg, the agent arrive rectangle early
		std::list<Constraint> constraint22;
		int Rg2e_t = getMahattanDistance(Rg2e_x, Rg2e_y, s2_x, s2_y);
		for (int i = 0; i <= k; i++) {
			constraint22.emplace_back(R2e_x * num_col + R2e_y, Rg2e_x * num_col + Rg2e_y, Rg2e_t + i, constraint_type::BARRIER);
		}
		multiConstraint2.push_back(constraint22);
	}

	return true;


	//exit(0);
}

// add a vertival modified barrier constraint
bool addModifiedVerticalBarrierConstraint(const std::vector<PathEntry>& path, int y,
	int Ri_x, int Rg_x, int Rg_t, int num_col,
	std::list<Constraint>& constraints)
{
	int sign = Ri_x < Rg_x ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_x - Rg_x);
	int t1 = -1;
	int t_min = std::max(Ri_t, 0);
	int t_max = std::min(Rg_t, (int)path.size() - 1);
	for (int t2 = t_min; t2 <= t_max; t2++)
	{
		int loc = (Ri_x + (t2 - Ri_t) * sign) * num_col + y;
		std::list<int>::const_iterator it = std::find(path[t2].locations.begin(), path[t2].locations.end(), loc);
		if (it == path[t2].locations.end() && t1 >= 0) // add constraints [t1, t2)
		{
			int loc1 = (Ri_x + (t1 - Ri_t) * sign) * num_col + y;
			int loc2 = (Ri_x + (t2 - 1 - Ri_t) * sign) * num_col + y;
			constraints.emplace_back(loc1, loc2, t2 - 1, constraint_type::BARRIER);
			t1 = -1;
			continue;
		}
		else if (it != path[t2].locations.end() && t1 < 0)
		{
			t1 = t2;
		}
		if (it != path[t2].locations.end() && t2 == t_max)
		{
			int loc1 = (Ri_x + (t1 - Ri_t) * sign) * num_col + y;
			constraints.emplace_back(loc1, loc, t2, constraint_type::BARRIER); // add constraints [t1, t2]
		}
	}
	if (constraints.empty())
	{
		// std::cout << "Fail to add modified barrier constraints!" << std::endl;
		return false;
	}
	else
		return true;
}

// add a vertival modified barrier constraint
bool addModifiedVerticalLongBarrierConstraint(const std::vector<PathEntry>& path, int y,
	int Ri_x, int Rg_x, int Rg_t, int num_col, int St,
	std::list<Constraint>& constraints, int k, const MDDLevels* kMDD)
{


	//std::cout << "vertical y:" << y<<" Rix:"<<Ri_x<<" Rgx:"<< Rg_x<<" t:"<<Rg_t << std::endl;

	//for (int i = 0; i < kMDD.size(); i++) {
	//	for (int l = 0; l < kMDD[i]->levels.size(); l++) {
	//		std::unordered_set<int>::iterator it;
	//		std::cout << "level " << l << ": ";
	//		for (it = kMDD[i]->levels[l].begin(); it != kMDD[i]->levels[l].end(); ++it) {
	//			std::cout << *it << "," << *it << " ";
	//		}
	//		std::cout << std::endl;
	//	}
	//}

	int sign = Ri_x < Rg_x ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_x - Rg_x);
	std::unordered_set<string> added;

	for (int t2 = Ri_t; t2 <= Rg_t; t2++)
	{
		int loc = (Ri_x + (t2 - Ri_t) * sign) * num_col + y;
		//std::cout << "target loc: " << loc / num_col << "," << loc % num_col << std::endl;
		for (int i = 0; i <= k; i++) {
			//std::cout << "add constraint on k= " << i << " t=" << t2 << ": ";
			if ((t2 + i < path.size())) {
				std::list<int>::const_iterator it = std::find(path[t2 + i].locations.begin(), path[t2 + i].locations.end(), loc);
				if (it != path[t2 + i].locations.end())
				{
						std::stringstream con;
						con << loc << t2 + i ;
						if (!added.count(con.str())) {

							constraints.emplace_back(loc, -1, t2 + i , constraint_type::VERTEX); // add constraints [t1, t2]
							//std::cout << "self mdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 << "|";
							added.insert(con.str());
						}




				}
				//std::cout << std::endl;
			}

			if (kMDD==NULL||t2 + i >= kMDD->size())
				continue;
			bool find = false;
			for (auto node : kMDD->at(t2 + i)){
			    if (node->location == loc){
			        find = true;
			        break;
			    }
			}
			if (find) {
					std::stringstream con;
					con << loc << t2 + i ;
					if (!added.count(con.str())) {
						constraints.emplace_back(loc, -1, t2 + i , constraint_type::VERTEX); // add constraints [t1, t2]
						//std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
						added.insert(con.str());
					}


			}

			//	}
			//}
			//std::cout << std::endl;
		}

	}
	if (constraints.empty())
	{
		// std::cout << "Fail to add modified barrier constraints!" << std::endl;
		return false;
	}
	else
		return true;
}


// add a horizontal modified barrier constraint
bool addModifiedHorizontalLongBarrierConstraint(const std::vector<PathEntry>& path, int x,
	int Ri_y, int Rg_y, int Rg_t, int num_col, int St,
	std::list<Constraint>& constraints, int k, const MDDLevels* kMDD)
{
	/*for (int t = 0; t < path.size(); t++) {
		std::cout << "(" << path.at(t).location / num_col << "," << path.at(t).location % num_col << ")";
		list<int>::const_iterator locs;
		for (locs = path.at(t).locations.begin(); locs != path.at(t).locations.end(); locs++)
		{
			cout << (*locs) << " ";
		}
		cout << "->";

	}
	std::cout << std::endl;*/
	//std::cout << "Horizontal x:" << x << " Riy:" << Ri_y << "Rgy:" << Rg_y << " t:" << Rg_t << std::endl;

	//for (int i = 0; i < kMDD.size(); i++) {
	//	for (int l = 0; l < kMDD[i]->levels.size(); l++) {
	//		std::unordered_set<int>::iterator it;
	//		std::cout << "level " << l << ": ";
	//		for (it = kMDD[i]->levels[l].begin(); it != kMDD[i]->levels[l].end(); ++it) {
	//			std::cout << *it << "," << *it << " ";
	//		}
	//		std::cout << std::endl;
	//	}
	//}

	int sign = Ri_y < Rg_y ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_y - Rg_y);
	int t1 = -1;
	bool overallFound = false;
	std::unordered_set<string> added;
	for (int t2 = Ri_t; t2 <= Rg_t; t2++)
	{
		int loc = (Ri_y + (t2 - Ri_t) * sign) + x * num_col;
		//std::cout << "target loc: " << loc / num_col << "," << loc % num_col << std::endl;
		for (int i = 0; i <= k; i++) {
			//std::cout << "add constraint on k= "<<i << " t=" << t2 << ": ";
			if ((t2 + i < path.size())) {

				std::list<int>::const_iterator it = std::find(path[t2 + i].locations.begin(), path[t2 + i].locations.end(), loc);

				if (it != path[t2 + i].locations.end())
				{

                    std::stringstream con;
                    con << loc << t2 + i;
                    if (!added.count(con.str())) {

                        constraints.emplace_back(loc, -1, t2 + i, constraint_type::VERTEX); // add constraints [t1, t2]
                        //std::cout << "self mdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 << "|";
                        added.insert(con.str());
                    }

				}
			//std::cout << std::endl;
			}


			//if (kMDD != NULL){
			//	//std::cout << "add constraint on k=" << i << " t=" << t2 << ": ";
			//	for (int mdd = 0; mdd < (*kMDD).size(); mdd++) {
			//		if ((t2 - St + i) >= (*kMDD)[mdd]->levels.size())
			//			continue;
			if (kMDD == NULL||t2 + i >= kMDD->size())
				continue;

            bool find = false;
            for (auto node : kMDD->at(t2 + i)){
                if (node->location == loc){
                    find = true;
                    break;
                }
            }
			if (find) {
					std::stringstream con;
					con << loc << t2 + i ;
					if (!added.count(con.str())) {
						constraints.emplace_back(loc, -1, t2 + i , constraint_type::VERTEX); // add constraints [t1, t2]
						//std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
						added.insert(con.str());
					}
			}


			//	}

			//}
			
			//std::cout << std::endl;
		}
	}
	if (constraints.empty())
	{
		// std::cout << "Fail to add modified barrier constraints!" << std::endl;
		return false;
	}
	else
		return true;
}

// add a vertival modified barrier constraint
bool addGeneralKVerticalBarrierConstraint(const std::vector<PathEntry>& path, int y,
                                              int Ri_x, int Rg_x, int Rg_t, int num_col, int St,
                                              std::list<Constraint>& constraints, int k, const MDDLevels* kMDD)
{


//    std::cout << "vertical y:" << y<<" Rix:"<<Ri_x<<" Rgx:"<< Rg_x<<" t:"<<Rg_t << std::endl;

    //for (int i = 0; i < kMDD.size(); i++) {
    //	for (int l = 0; l < kMDD[i]->levels.size(); l++) {
    //		std::unordered_set<int>::iterator it;
    //		std::cout << "level " << l << ": ";
    //		for (it = kMDD[i]->levels[l].begin(); it != kMDD[i]->levels[l].end(); ++it) {
    //			std::cout << *it << "," << *it << " ";
    //		}
    //		std::cout << std::endl;
    //	}
    //}

    int sign = Ri_x < Rg_x ? 1 : -1;
    int Ri_t = Rg_t - abs(Ri_x - Rg_x);
    int extended = k/2;

    for (int t2 = Ri_t; t2 <= Rg_t; t2++)
    {
        int loc = (Ri_x + (t2 - Ri_t) * sign) * num_col + y;
//        std::cout << "target loc: " << loc / num_col << "," << loc % num_col << std::endl;
        for (int i = 0; i <= k; i++) {
            if (t2 + i < kMDD->size()){
                bool find = false;
                for (auto node : kMDD->at(t2 + i)){
                    if (node->location == loc){
                        find = true;
                        break;
                    }
                }
                if (find) {
                    constraints.emplace_back(loc, -1, t2 + i , constraint_type::VERTEX); // add constraints [t1, t2]
//                    std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i << "|";
                }
            }
        }
//        std::cout <<endl;

    }

    for (int extend = 1;  extend <= extended; extend++){
        int t_left = Ri_t + extend;
        int t_right = Rg_t + extend;
        int range = k - extend * 2;
        int loc_left = (Ri_x - extend * sign) * num_col + y;
        int loc_right = (Rg_x + extend * sign) * num_col + y;
        if (range < 0){
            break;
        }

        for (int i = 0; i <= range; i++) {

            if (t_left + i < kMDD->size()){
                bool find = false;
                for (auto node : kMDD->at(t_left + i)){
                    if (node->location == loc_left){
                        find = true;
                        break;
                    }
                }
                if (find) {
                    constraints.emplace_back(loc_left, -1, t_left + i , constraint_type::VERTEX); // add constraints [t1, t2]
                    //std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
                }
            }

            if (t_right + i < kMDD->size()){
                bool find = false;
                for (auto node : kMDD->at(t_right + i)){
                    if (node->location == loc_right){
                        find = true;
                        break;
                    }
                }
                if (find) {
                    constraints.emplace_back(loc_right, -1, t_right + i , constraint_type::VERTEX); // add constraints [t1, t2]
                    //std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
                }
            }
        }

    }

    if (constraints.empty())
    {
        // std::cout << "Fail to add modified barrier constraints!" << std::endl;
        return false;
    }
    else
        return true;
}


// add a horizontal modified barrier constraint
bool addGeneralKHorizontalBarrierConstraint(const std::vector<PathEntry>& path, int x,
                                                int Ri_y, int Rg_y, int Rg_t, int num_col, int St,
                                                std::list<Constraint>& constraints, int k, const MDDLevels* kMDD)
{
    /*for (int t = 0; t < path.size(); t++) {
        std::cout << "(" << path.at(t).location / num_col << "," << path.at(t).location % num_col << ")";
        list<int>::const_iterator locs;
        for (locs = path.at(t).locations.begin(); locs != path.at(t).locations.end(); locs++)
        {
            cout << (*locs) << " ";
        }
        cout << "->";

    }
    std::cout << std::endl;*/
//    std::cout << "Horizontal x:" << x << " Riy:" << Ri_y << "Rgy:" << Rg_y << " t:" << Rg_t << std::endl;

    //for (int i = 0; i < kMDD.size(); i++) {
    //	for (int l = 0; l < kMDD[i]->levels.size(); l++) {
    //		std::unordered_set<int>::iterator it;
    //		std::cout << "level " << l << ": ";
    //		for (it = kMDD[i]->levels[l].begin(); it != kMDD[i]->levels[l].end(); ++it) {
    //			std::cout << *it << "," << *it << " ";
    //		}
    //		std::cout << std::endl;
    //	}
    //}

    int sign = Ri_y < Rg_y ? 1 : -1;
    int Ri_t = Rg_t - abs(Ri_y - Rg_y);
    int extended = k/2;
    for (int t2 = Ri_t; t2 <= Rg_t; t2++)
    {
        int loc = (Ri_y + (t2 - Ri_t) * sign) + x * num_col;
//        std::cout << "target loc: " << loc / num_col << "," << loc % num_col << std::endl;
        for (int i = 0; i <= k; i++) {

            if (t2 + i < kMDD->size()){
                bool find = false;
                for (auto node : kMDD->at(t2 + i)){
                    if (node->location == loc){
                        find = true;
                        break;
                    }
                }
                if (find) {
                    constraints.emplace_back(loc, -1, t2 + i , constraint_type::VERTEX); // add constraints [t1, t2]
//                        std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i << "|";
                }
            }

        }
//        std::cout <<endl;

    }

    for (int extend = 1;  extend <= extended; extend++){
        int t_left = Ri_t + extend;
        int t_right = Rg_t + extend;
        int range = k - extend * 2;
        int loc_left = (Ri_y - extend * sign) + x * num_col ;
        int loc_right = (Rg_y + extend * sign) + x * num_col ;
        if (range < 0){
            break;
        }

        for (int i = 0; i <= range; i++) {

            if (t_left + i < kMDD->size()){
                bool find = false;
                for (auto node : kMDD->at(t_left + i)){
                    if (node->location == loc_left){
                        find = true;
                        break;
                    }
                }
                if (find) {
                    constraints.emplace_back(loc_left, -1, t_left + i , constraint_type::VERTEX); // add constraints [t1, t2]
                    //std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
                }
            }

            if (t_right + i < kMDD->size()){
                bool find = false;
                for (auto node : kMDD->at(t_right + i)){
                    if (node->location == loc_right){
                        find = true;
                        break;
                    }
                }
                if (find) {
                    constraints.emplace_back(loc_right, -1, t_right + i , constraint_type::VERTEX); // add constraints [t1, t2]
                    //std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
                }
            }
        }

    }


    if (constraints.empty())
    {
        // std::cout << "Fail to add modified barrier constraints!" << std::endl;
        return false;
    }
    else
        return true;
}

	// add a vertival modified barrier constraint
	bool add4WayModifiedVerticalLongBarrierConstraint(const std::vector<PathEntry>& path, int y,
		int Ri_x, int Rg_x, int Rg_t, int num_col, int St,
		std::list<Constraint>& constraints, int k)
	{
		for (int i = 0; i <= k; i++) {
			constraints.emplace_back(Ri_x * num_col + y, Rg_x * num_col + y, Rg_t+i, constraint_type::BARRIER);
		}

		
		if (constraints.empty())
		{
			// std::cout << "Fail to add modified barrier constraints!" << std::endl;
			return false;
		}
		else
			return true;
	}


	// add a horizontal modified barrier constraint
	bool add4WayModifiedHorizontalLongBarrierConstraint(const std::vector<PathEntry>& path, int x,
		int Ri_y, int Rg_y, int Rg_t, int num_col, int St,
		std::list<Constraint>& constraints, int k)
	{
		

		for (int i = 0; i <= k; i++) {
			constraints.emplace_back(x * num_col + Ri_y, x * num_col + Rg_y, Rg_t + i, constraint_type::BARRIER);
		}

		if (constraints.empty())
		{
			// std::cout << "Fail to add modified barrier constraints!" << std::endl;
			return false;
		}
		else
			return true;


	}

	// add a vertival modified barrier constraint
bool addFlippedVerticalLongBarrierConstraint(const std::vector<PathEntry>& path, int y,
	vector<int> vertical,vector<int> verticalMin,vector<int> verticalMax, int num_col, int St,
	std::list<Constraint>& constraints, int k, MDDPath* kMDD)
	{
		//std::cout << "vertical y:" << y<<" Rix:"<<Ri_x<<" Rgx:"<< Rg_x<<" t:"<<Rg_t << std::endl;

		std::unordered_set<string> added;

		for (int count = 0;count<vertical.size();count++)
		{
			int loc = vertical[count] * num_col + y;
			int tMin = verticalMin[count];
			int tMax = verticalMax[count];
			if (tMin > tMax)
				continue;
			//std::cout << "target loc: " << loc / num_col << "," << loc % num_col << std::endl;
			for (int t = tMin; t <= tMax; t++) {
				//std::cout << "add constraint on t=" << t << ": ";
				if ((t < path.size())) {
					std::list<int>::const_iterator it = std::find(path[t].locations.begin(), path[t].locations.end(), loc);
					if (it != path[t].locations.end())
					{
						std::stringstream con;
						con << loc << t;
						if (!added.count(con.str())) {
							//constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]

							if(t==tMax)
								constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]
							else
								constraints.emplace_back(loc, t, tMax, constraint_type::RANGE); // add constraints [t1, t2]
							
																								  //std::cout << "self mdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t << "|";
							added.insert(con.str());
							break;
						}
					}
				}
				//else {
				//	std::cout << "t: " << t << " > " << path.size();
				//}
				//std::cout << std::endl;

				if (kMDD == NULL || t >= kMDD->levels.size())
					continue;
				if ((kMDD)->levels[t].count(loc)) {
					std::stringstream con;
					con << loc << t;
					if (!added.count(con.str())) {
						//constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]

						if (t == tMax)
							constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]
						else
							constraints.emplace_back(loc, t, tMax, constraint_type::RANGE); // add constraints [t1, t2]
						////std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
						added.insert(con.str());
						break;
					}
				}
			}

		}
		if (constraints.empty())
		{
			// std::cout << "Fail to add modified barrier constraints!" << std::endl;
			return false;
		}
		else
			return true;
	}


	// add a horizontal modified barrier constraint
bool addFlippedHorizontalLongBarrierConstraint(const std::vector<PathEntry>& path, int x,
	vector<int> horizontal, vector<int> horizontalMin, vector<int> horizontalMax, int num_col, int St,
	std::list<Constraint>& constraints, int k, MDDPath* kMDD)
	{
		//std::cout << "Horizontal x:" << x << " Riy:" << Ri_y << "Rgy:" << Rg_y << " t:" << Rg_t << std::endl;

		std::unordered_set<string> added;
		for (int count = 0; count < horizontal.size(); count++)
		{
			int loc = x*num_col + horizontal[count] ;
			int tMin = horizontalMin[count];
			int tMax = horizontalMax[count];
			if (tMin > tMax)
				continue;
			//std::cout << "target loc: " << loc / num_col << "," << loc % num_col << std::endl;
			for (int t = tMin; t <= tMax; t++) {
				//std::cout << "add constraint on t=" << t << ": ";
				if (t < path.size()) {
					std::list<int>::const_iterator it = std::find(path[t].locations.begin(), path[t].locations.end(), loc);
					if (it != path[t].locations.end())
					{
						std::stringstream con;
						con << loc << t;
						if (!added.count(con.str())) {
							//constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]

							if (t == tMax)
								constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]
							else
								constraints.emplace_back(loc, t, tMax, constraint_type::RANGE); // add constraints [t1, t2]
							//std::cout << "self mdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t << "|";
							added.insert(con.str());
							break;
						}
					}
					
				}
				//else {
				//	std::cout << "t: " << t << " > " << path.size();
				//}
				//std::cout << std::endl;

				if (kMDD == NULL || t >= kMDD->levels.size())
					continue;
				if ((kMDD)->levels[t].count(loc)) {
					std::stringstream con;
					con << loc << t;
					if (!added.count(con.str())) {
						//constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]

						if (t == tMax)
							constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]
						else
							constraints.emplace_back(loc, t, tMax, constraint_type::RANGE); // add constraints [t1, t2]
						////std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
						added.insert(con.str());
						break;
					}
				}
			}
		}
		if (constraints.empty())
		{
			// std::cout << "Fail to add modified barrier constraints!" << std::endl;
			return false;
		}
		else
			return true;


	}
