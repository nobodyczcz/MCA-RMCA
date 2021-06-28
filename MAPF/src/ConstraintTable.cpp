#include "ConstraintTable.h"
void ConstraintTable::insert(int loc, int t_min, int t_max)
{
	if (t_max == 0) {
		CT_Single[loc].emplace(t_min);
		t_max = t_min + 1;
	}
	else {
		CT[loc].emplace_back(t_min, t_max);
	}
	
	if (loc == goal_location && t_max > length_min)
	{
		length_min = t_max;
	}
	if (t_max < INT_MAX && t_max > latest_timestep)
	{
		latest_timestep = t_max;
	}
}

void ConstraintTable::insert(std::list<Constraint> &constraints, int agent_id, int num_col, int map_size) {
    for (auto constraint : constraints)
    {
        int x, y, z;
        constraint_type type;
        tie(x, y, z, type) = constraint;
        if (type == constraint_type::RANGE) // time range constraint
        {
            this->insert(x, y, z + 1);
        }
        else if (type == constraint_type::BARRIER) // barrier constraint
        {
            int x1 = x / num_col, y1 = x % num_col;
            int x2 = y / num_col, y2 = y % num_col;
            if (x1 == x2)
            {
                if (y1 < y2)
                    for (int i = 0; i <= std::min(y2 - y1, z); i++)
                    {
                        this->insert(x1 * num_col + y2 - i, z - i, 0);
                    }
                else
                    for (int i = 0; i <= std::min(y1 - y2, z); i++)
                    {

                        this->insert(x1 * num_col + y2 + i, z - i, 0);
                    }
            }
            else // y1== y2
            {
                if (x1 < x2)
                    for (int i = 0; i <= std::min(x2 - x1, z); i++)
                    {

                        this->insert((x2 - i) * num_col + y1, z - i, 0);
                    }
                else
                    for (int i = 0; i <= std::min(x1 - x2, z); i++)
                    {

                        this->insert((x2 + i) * num_col + y1, z - i, 0);
                    }
            }
        }
        else if (type == constraint_type::LENGTH)
        {
            if (x < 0 && y == agent_id)
            { // <-1, agent_id, t>: path of agent_id should be of length at least t + 1
                this->length_min = max(this->length_min, z + 1);

            }
            else if (x >= 0 && y == agent_id)
            { // <loc, agent_id, t>: path of agent_id should be of length at most t !!not possible as y==agent will jump findpath
                this->length_max = min(this->length_max, z);

            }
            else if (x >= 0 && y != agent_id)
            { // <loc, agent_id, t>: any other agent cannot be at loc at or after timestep t
                this->insert(x, z, INT_MAX);
            }
        }
        else if (type == constraint_type::VERTEX)
        {
            this->insert(x, z, 0);
        }
        else // edge
        {
            this->insert(x * map_size + y, z, 0);
        }
    }
}

bool ConstraintTable::insert_path(int agent_id, const Path& path)
{
    for (int timestep = 0; timestep < (int)path.size(); timestep++)
    {
        int loc = path[timestep].location;
        if (loc == -1)
            continue;

//        if (CT_paths[loc].empty())
//            CT_paths[loc].resize(length_max  + 1, -1);

//        if (CT_paths[loc].count(timestep)) {
//            if (CT_paths[loc][timestep] != -1)
//                cout<<CT_paths[loc][timestep]<<endl;
//            assert(CT_paths[loc][timestep] == -1);
//        }
        if(CT_paths[loc].count(timestep)){
//            cout<<CT_paths[loc][timestep]<<endl;
            assert(CT_paths[loc][timestep]==-1);
        }
        CT_paths[loc][timestep] = agent_id;


    }
    return true;
}

void ConstraintTable::delete_path(int agent_id, const Path& path)
{
    for (int timestep = 0; timestep < (int)path.size(); timestep++)
    {
        int loc = path[timestep].location;

//        if (CT_paths[loc].count(timestep)){
//            assert(CT_paths[loc][timestep] == agent_id);
//        }
        CT_paths[loc][timestep] = -1;
    }
}

bool ConstraintTable::is_constrained(int loc, int t)
{
	if (CT_Single.count(loc)) {
		if (CT_Single[loc].count(t)) {
			return true;
		}
	}
	auto it = CT.find(loc);
	if (it == CT.end())
	{
		return false;
	}
	for (auto constraint : it->second)
	{
		if (constraint.first <= t && t < constraint.second)
			return true;
	}

	return false;
}
bool ConstraintTable::is_path_constrained(int agent_id, int loc, int pre_loc, int t)
{
    if (CT_docking[loc].first!=-1 && CT_docking[loc].first!=agent_id && t>= CT_docking[loc].second) {
//        cout<<CT_docking[loc].first<<" docking"<<endl;
        return true;
    }

    if (CT_paths[loc].count(t) )
    {
        if (CT_paths[loc][t] >= 0 && agent_id != CT_paths[loc][t]) {
//            cout<<CT_paths[loc][t] <<" vertex"<<endl;
            return true;
        }
    }

    if (t-1 >= 0 && CT_paths[pre_loc].count(t) && CT_paths[loc].count(t-1))
    {
        if (CT_paths[loc][t-1] != -1 && CT_paths[loc][t-1] != agent_id && CT_paths[loc][t-1]>=0 && CT_paths[loc][t-1] == CT_paths[pre_loc][t]){
//            cout<<CT_paths[loc][t-1]<<endl;
            return true; //edge conflict
        }
    }


    return false;
}

bool ConstraintTable::is_dock_safe(int agent_id, int loc, int t)
{

    for (auto i : CT_paths[loc]){
        if (i.second != -1 && i.second != agent_id && t <= i.first )
            return false;
    }

    return true;
}
