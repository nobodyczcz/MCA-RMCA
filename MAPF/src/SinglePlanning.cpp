#include "SinglePlanning.h"

#include <iostream>


void SinglePlanning::updatePath(LLNode* goal)
{
	path.resize(goal->timestep + 1-start_time);
	LLNode* curr = goal;
	num_of_conf = goal->num_internal_conf;
	for(int t = goal->timestep; t >= start_time; t--)
	{
	    int index = t-start_time;


		path[index].location = curr->loc;
		path[index].heading = curr->heading;
		path[index].timeStep = t;
        if(curr->parent == NULL)
            break;
		curr = curr->parent;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// return true if a path found (and updates vector<int> path) or false if no path exists

bool SinglePlanning::search(bool docking )
{
    Time::time_point start_clock = Time::now();

    LL_num_expanded = 0;
    LL_num_generated = 0;

	hashtable_t::iterator it;  // will be used for find()


	 // generate start and add it to the OPEN list
	LLNode* start;
	start = new LLNode(start_location, 0, ml.getDistance(start_location, goal_location), nullptr, start_time, 0, false);

    LL_num_generated++;
	start->open_handle = open_list.push(start);
	start->focal_handle = focal_list.push(start);
	start->in_openlist = true;
	start->time_generated = 0;



	allNodes_table.insert(start);
	min_f_val = start->getFVal();

    focal_threshold = std::max(double(min_end_time-start_time), double(f_w * min_f_val));
    assert(focal_threshold>=min_f_val);


    int time_generated = 0;
	int time_check_count = 0;
    fsec runtime;
	/*for (int h = 0; h < my_heuristic.size();h++) {
		for (int heading = 0; heading<5;heading++)
			std::cout << "(" << h << ": heading:"<<-1 <<": "<< my_heuristic[h].heading[-1] << ")";
	}*/

	while (!focal_list.empty()) 
	{
		if (LL_num_generated / 10000 > time_check_count && time_limit != 0) {
			runtime = Time::now() - start_clock;
			time_check_count = LL_num_generated / 10000;
			if (runtime.count() > time_limit) {
                releaseClosedListNodes(&allNodes_table);

                open_list.clear();
                focal_list.clear();

                allNodes_table.clear();
				return false;
			}
		}

		LLNode* curr = focal_list.top(); focal_list.pop();
		open_list.erase(curr->open_handle);

		curr->in_openlist = false;
        LL_num_expanded++;
		//cout << "focal size " << focal_list.size() << endl;
		//cout << "goal_location: " << goal_location << " curr time: " << curr->timestep << " length_min: " << constraintTable.length_min << endl;
		// check if the popped node is a goal
		if ((curr->loc == goal_location ) && curr->timestep>=min_end_time && (!docking || constraintTable.is_dock_safe(agent_id,curr->loc,curr->timestep)))
		{


			updatePath(curr);

			releaseClosedListNodes(&allNodes_table);

			open_list.clear();
			focal_list.clear();

			allNodes_table.clear();
			return true;
			//}
		}


        vector<pair<int, int>> transitions = ml.get_transitions(curr->loc, -1,false);


		for (const auto& move : transitions)
		{
			int next_id = move.first;
			time_generated += 1;
			int next_timestep = curr->timestep + 1;


			if (!constraintTable.is_path_constrained(agent_id, next_id,curr->loc, next_timestep))
			{

				int next_g_val = curr->g_val + 1;
				int next_heading;

                int next_h_val;

                next_h_val = ml.getDistance(next_id,goal_location);


                // generate (maybe temporary) node
				auto next = new LLNode(next_id, next_g_val, next_h_val,	curr, next_timestep, 0, false);
				next->heading = next_heading;
				next->time_generated = time_generated;

				// try to retrieve it from the hash table
				it = allNodes_table.find(next);
				if (it == allNodes_table.end())
				{

					next->open_handle = open_list.push(next);
					next->in_openlist = true;
                    LL_num_generated++;
					if (next->getFVal() <= focal_threshold)
					{
						next->focal_handle = focal_list.push(next);
					}

					allNodes_table.insert(next);
				}
				else
				{  // update existing node's if needed (only in the open_list)
					delete(next);  // not needed anymore -- we already generated it before
					LLNode* existing_next = (*it);

					if (existing_next->in_openlist)
					{  // if its in the open list
						if (existing_next->getFVal() > next_g_val + next_h_val)
						{
							// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
							bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
							bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
							bool update_open = false;
							if ((next_g_val + next_h_val) <= focal_threshold)
							{  // if the new f-val qualify to be in FOCAL
								if (existing_next->getFVal() > focal_threshold)
									add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
								else
									update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
							}
							if (existing_next->getFVal() > next_g_val + next_h_val)
								update_open = true;
							// update existing node
							existing_next->g_val = next_g_val;
							existing_next->h_val = next_h_val;
							existing_next->parent = curr;
							if (update_open) 
								open_list.increase(existing_next->open_handle);  // increase because f-val improved
							if (add_to_focal) 
								existing_next->focal_handle = focal_list.push(existing_next);
							if (update_in_focal) 
								focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down
						}				
					}
					else 
					{  // if its in the closed list (reopen)
						if (existing_next->getFVal() > next_g_val + next_h_val )
						{
							// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
							existing_next->g_val = next_g_val;
							existing_next->h_val = next_h_val;
							existing_next->parent = curr;
							existing_next->open_handle = open_list.push(existing_next);
							existing_next->in_openlist = true;
							if (existing_next->getFVal() <= focal_threshold)
                                existing_next->focal_handle = focal_list.push(existing_next);
						}
					}  // end update a node in closed list
				}  // end update an existing node
			}// end if case forthe move is legal
		}  // end for loop that generates successors
		//cout << "focal list size"<<focal_list.size() << endl;
		// update FOCAL if min f-val increased
		if (open_list.empty())  // in case OPEN is empty, no path found
			break;
		LLNode* open_head = open_list.top();

        //assert(open_head->getFVal() >= min_f_val);
		if (open_head->getFVal() > min_f_val) 
		{
			min_f_val = open_head->getFVal();
			double new_focal_threshold = std::max(f_w * min_f_val, min_f_val);
            if (new_focal_threshold > focal_threshold)
            {
                for (LLNode* n : open_list)
                {

                    if (n->getFVal() > focal_threshold && n->getFVal() <= new_focal_threshold)
                    {
                        n->focal_handle = focal_list.push(n);
                    }
                }
                focal_threshold = new_focal_threshold;
            }
		}


	}  // end while loop

//	assert(min_f_val >= constraintTable.length_max);
	  // no path found
	releaseClosedListNodes(&allNodes_table);
	open_list.clear();
	focal_list.clear();
	allNodes_table.clear();
	return false;
}


inline void SinglePlanning::releaseClosedListNodes(hashtable_t* allNodes_table)
{

	hashtable_t::iterator it;
	for (it = allNodes_table->begin(); it != allNodes_table->end(); ++it) {

			delete (*it);
	}
}


SinglePlanning::SinglePlanning(MapLoaderCost& ml,int agent_id, int start, int goal, int start_time, int min_end_time, double f_w, float time_limit, options option, ConstraintTable& constraintTable):
    ml(ml),agent_id(agent_id),start_time(start_time),min_end_time(min_end_time),constraintTable(constraintTable)
{


    this->screen = screen;
    this->time_limit= time_limit;
	this->f_w = f_w;
	this->start_location = start;
	this->goal_location = goal;

	this->map_size = ml.cols*ml.rows;

	this->LL_num_expanded = 0;
	this->LL_num_generated = 0;

	this->focal_threshold = 0;
	this->min_f_val = 0;

	this->num_col = ml.cols;

}
