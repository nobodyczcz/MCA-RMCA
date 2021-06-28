//=======================================================================

#include "agents_loader.h"
#include <string>
#include <cstring>
#include <iostream>
#include <cassert>
#include <fstream>
#include<boost/tokenizer.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <utility>
#include <algorithm>  // for remove_if
#include <ctime>
using namespace boost;
using namespace std;


int RANDOM_WALK_STEPS = 100000;


AgentsLoader::AgentsLoader(int number_of_agent){
    this->num_of_agents = number_of_agent;
    initial_locations.resize(number_of_agent);
    goal_locations.resize(number_of_agent);
    headings.resize(number_of_agent);


}

AgentsLoader::AgentsLoader(string fname, const MapLoader &ml, int agentsNum = 0){
  string line;

  ifstream myfile (fname.c_str());

  if (myfile.is_open()) {
    getline (myfile,line);
    char_separator<char> sep(",");
    tokenizer< char_separator<char> > tok(line, sep);
    tokenizer< char_separator<char> >::iterator beg=tok.begin();
    this->num_of_agents = atoi ( (*beg).c_str() );
    //    cout << "#AG=" << num_of_agents << endl;
    for (int i=0; i<num_of_agents; i++) {
      getline (myfile, line);
      tokenizer< char_separator<char> > col_tok(line, sep);
      tokenizer< char_separator<char> >::iterator c_beg=col_tok.begin();
      pair<int,int> curr_pair;
      // read start [row,col] for agent i
      curr_pair.first = atoi ( (*c_beg).c_str() );
      c_beg++;
      curr_pair.second = atoi ( (*c_beg).c_str() );
      //      cout << "AGENT" << i << ":   START[" << curr_pair.first << "," << curr_pair.second << "] ; ";
      this->initial_locations.push_back(curr_pair);
      // read goal [row,col] for agent i
      c_beg++;
      curr_pair.first = atoi ( (*c_beg).c_str() );
      c_beg++;
      curr_pair.second = atoi ( (*c_beg).c_str() );
      //      cout << "GOAL[" << curr_pair.first << "," << curr_pair.second << "]" << endl;
      this->goal_locations.resize(1);
      this->min_end_time.resize(1);
      this->goal_locations[0].push_back(curr_pair);
	  this->headings.push_back(-1);
	  this->min_end_time[0].push_back(0);
	  this->done.push_back(false);



        // read max velocity and accelration for agent i
     /* c_beg++;
      this->max_v.push_back(atof((*c_beg).c_str()));
      c_beg++;
      this->max_w.push_back(atof((*c_beg).c_str()));
      c_beg++;
      this->max_a.push_back(atof((*c_beg).c_str()));*/
    }
    myfile.close();
  } 
  else if(agentsNum > 0)//Generate agents randomly
  {
	  this->num_of_agents = agentsNum;
	  vector<bool> starts(ml.rows * ml.cols, false);
	  vector<bool> goals(ml.rows * ml.cols, false);
	  // Choose random start locations
	  for (int k = 0; k < agentsNum; k++)
	  {
		  int x = rand() % ml.rows, y = rand() % ml.cols;
		  int start = x * ml.cols +y;
		  if (!ml.my_map[start] && !starts[start])
		  {
				// update start
				this->initial_locations.push_back(make_pair(x,y));
				starts[start] = true;

				// random walk
				int loc = start;
				bool* temp_map = new bool[ml.rows * ml.cols];
				for (int walk = 0; walk < RANDOM_WALK_STEPS; walk++)
				{
					int directions[] = {0, 1, 2, 3, 4};
					random_shuffle(directions, directions + 5);
					int i = 0;
					for(; i< 5; i++)
					{
						int next_loc = loc + ml.moves_offset[directions[i]];
						if (0 <= next_loc && next_loc < ml.rows * ml.cols &&! ml.my_map[next_loc])
						{
							loc = next_loc;
							break;
						}
					}
					if (i == 5)
					{
						cout << "--------------ERROR!-----------------" << endl;
						system("pause");
					}
				}
				// find goal
				bool flag = false;
				int goal = loc;
				while (!flag)
				{
					int directions[] = { 0, 1, 2, 3, 4 };
					random_shuffle(directions, directions + 5);
					int i = 0;
					for (; i< 5; i++)
					{
						int next_loc = goal + ml.moves_offset[directions[i]];
						if (0 <= next_loc && next_loc < ml.rows * ml.cols && !ml.my_map[next_loc])
						{
							goal = next_loc;
							break;
						}
					}
					if (i == 5)
					{
						cout << "--------------ERROR!-----------------" << endl;
						system("pause");
					}
					flag = true;
					if (goals[goal])
						flag = false;
				}
				//update goal
              this->goal_locations.resize(1);
              this->min_end_time.resize(1);
				this->goal_locations[0].push_back(make_pair(goal / ml.cols, goal % ml.cols));
				goals[goal] = true;
				this->headings.push_back(-1);
                this->done.push_back(false);
                this->min_end_time[0].push_back(0);


              // update others
				/*this->max_v.push_back(1);
				this->max_w.push_back(1);
				this->max_a.push_back(1);*/
		  }
		  else
		  {
			  k--;
		  }
	  }
	  saveToFile(fname);
  }
  else
  {
	  cerr << "Agent file " << fname << " not found." << std::endl;
	  exit(10);
  }
}

void AgentsLoader::printAgentsInitGoal () {
  cout << "AGENTS:" << endl;;
  for (int i=0; i<num_of_agents; i++) {
    cout << "Agent" << i << " : I=(" << initial_locations[i].first << "," << initial_locations[i].second << ") ; G=";
    for(int g = 0;  g<goal_locations[i].size();g++){
        cout<<"("<<goal_locations[i][g].first << "," << goal_locations[i][g].second << ") min: " <<min_end_time[i][g]<<", ";
    }
    cout << " done: " << done[i] <<", Heading: "<<headings[i]<< endl;
  }
  cout << endl;
}

AgentsLoader::~AgentsLoader() {
  // vectors are on stack, so they are freed automatically
}

// create an empty object
AgentsLoader::AgentsLoader() {
  num_of_agents = 0;
}

// returns the agents' ids if they occupy [row,col] (first for start, second for goal)
pair<int, int> AgentsLoader::agentStartOrGoalAt(int row, int col) {
  int f = -1;
  int s = -1;
  for (vector< pair<int, int> >::iterator it = initial_locations.begin(); it != initial_locations.end(); ++it)
    if ( it->first == row && it->second == col )
      f = std::distance(initial_locations.begin(), it);
  for (vector< pair<int, int> >::iterator it = goal_locations.back().begin(); it != goal_locations.back().end(); ++it)
    if ( it->first == row && it->second == col )
      s = std::distance(goal_locations.back().begin(), it);
  return make_pair(f, s);
}


void AgentsLoader::clearLocationFromAgents(int row, int col) {
  pair<int, int> idxs = agentStartOrGoalAt(row, col);
  if ( idxs.first != -1 ) {  // remove the agent who's start is at [row,col]
    initial_locations.erase( initial_locations.begin() + idxs.first );
    goal_locations.erase ( goal_locations.begin() + idxs.first );
    num_of_agents--;
  }
  idxs = agentStartOrGoalAt(row, col);
  if ( idxs.second != -1 ) {  // remove the agent who's goal is at [row,col]
    initial_locations.erase( initial_locations.begin() + idxs.second );
    goal_locations.erase( goal_locations.begin() + idxs.second );
    num_of_agents--;
  }
}


// add an agent
void AgentsLoader::addAgent(int start_row, int start_col, int goal_row, int goal_col,int min_time,int finish, int heading) {
    this->initial_locations.push_back(make_pair(start_row, start_col));
    this->goal_locations.push_back(vector<pair<int, int>>());
    this->goal_locations.back().push_back(make_pair(goal_row, goal_col));
    this->headings.push_back(heading);
    this->min_end_time.push_back((vector<int>()));
    this->min_end_time.back().push_back(min_time);
    this->done.push_back(finish);
    num_of_agents++;
}
void AgentsLoader::addAgent(int start_row, int start_col, vector<pair<int,int>> goal,vector<int> min_time,int finish, int heading) {
    this->initial_locations.push_back(make_pair(start_row, start_col));
    this->goal_locations.push_back(goal);
    this->headings.push_back(heading);
    this->min_end_time.push_back(min_time);
    this->done.push_back(finish);
    num_of_agents++;
}

void AgentsLoader::saveToFile(std::string fname) {
  ofstream myfile;
  myfile.open(fname);
  myfile << num_of_agents << endl;
  for (int i = 0; i < num_of_agents; i++)
    myfile << initial_locations[i].first << "," << initial_locations[i].second << ","
           << goal_locations[i].back().first << "," << goal_locations[i].back().second << ","
		   /*<< max_v[i] << "," << max_a[i] << "," << max_w[i] << ","*/  << endl;
  myfile.close();
}

void AgentsLoader::clear(){
    initial_locations.clear();
    goal_locations.clear();
    min_end_time.clear();
    done.clear();
    headings.clear();
    num_of_agents = 0;

}
