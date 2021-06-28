// Load's agents' init and goal states.
// First line: number of agents
// Second line and onward, (x_init,y_init),(x_goal,y_goal) of each agent (one per line)

#ifndef AGENTSLOADER_H
#define AGENTSLOADER_H

#include <string>
#include <vector>
#include <utility>
#include "map_loader.h"


using namespace std;

class AgentsLoader {
 public:
  int num_of_agents;
  vector< pair<int, int> > initial_locations;
  vector< vector<pair<int, int>> > goal_locations;
  vector<vector<int>> min_end_time;
  vector<bool> done;
  vector<int> headings;
  vector<double> max_v;  // entry [i] is the max translational velocity for agent i
  vector<double> max_w;  // entry [i] is the max rotational velocity for agent i
  vector<double> max_a;  // entry [i] is the max accelration for agent i
  AgentsLoader(const std::string fname, const MapLoader &ml, int agentsNum);
  AgentsLoader();
  AgentsLoader(int number_of_agent);

  void addAgent ( int start_row, int start_col, int goal_row, int goal_col,int min_time = 0,int finish = false, int heading = -1);
  void addAgent(int start_row, int start_col, vector<pair<int,int>> goal,vector<int> min_time,int finish = false, int heading = -1);
  void clear();

  void printAgentsInitGoal ();
  void saveToFile(const std::string fname);
  pair<int, int> agentStartOrGoalAt(int row, int col);
  void clearLocationFromAgents(int row, int col);
  ~AgentsLoader();
};

#endif
