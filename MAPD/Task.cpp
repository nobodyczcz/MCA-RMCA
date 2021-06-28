//
// Created by Zhe Chen on 10/6/20.
//

#include "Task.h"
#include <sstream>

Task::Task(int id,int init_t, int init_loc, int goal_loc){
    task_id = id;
    initial_time = init_t;
    initial_location = init_loc;
    goal_location = goal_loc;
}

TaskLoader::TaskLoader(const std::string fname, MapLoaderCost &ml){
    string line;

    ifstream myfile (fname.c_str());

    if (myfile.is_open()) {
        getline (myfile,line);
        char_separator<char> sep(",");
        tokenizer< char_separator<char> > tok(line, sep);
        tokenizer< char_separator<char> >::iterator beg=tok.begin();
        this->num_of_tasks = atoi ( (*beg).c_str() );
        //    cout << "#AG=" << num_of_agents << endl;
        for (int i=0; i<num_of_tasks; i++) {
            getline (myfile, line);
            tokenizer< char_separator<char> > col_tok(line, sep);
            tokenizer< char_separator<char> >::iterator c_beg=col_tok.begin();
            pair<int,int> curr_pair;
            // read start [row,col] for agent i
            curr_pair.first = atoi ( (*c_beg).c_str() );
            c_beg++;
            curr_pair.second = atoi ( (*c_beg).c_str() );
            int init_loc = curr_pair.first * ml.cols + curr_pair.second;

            //      cout << "AGENT" << i << ":   START[" << curr_pair.first << "," << curr_pair.second << "] ; ";
            // read goal [row,col] for agent i
            c_beg++;
            curr_pair.first = atoi ( (*c_beg).c_str() );
            c_beg++;
            curr_pair.second = atoi ( (*c_beg).c_str() );
            int goal_loc = curr_pair.first * ml.cols + curr_pair.second;

            c_beg++;
            int init_time = atoi ( (*c_beg).c_str() );
            Task* new_task = new Task(i,init_time,init_loc,goal_loc);
            new_task->ideal_end_time = ml.getDistance(init_loc,goal_loc) + init_time;
            all_tasks.push(new_task);

        }
        myfile.close();
    }
}

void TaskLoader::loadKiva(const std::string fname, MapLoaderCost &ml){
    string line;
    ifstream myfile(fname.c_str());
    if (!myfile.is_open())
    {
        assert(false);
        cerr << "Task file not found." << endl;
        return;
    }
    //read file
    stringstream ss;
    int task_num;
    getline(myfile, line);
    ss << line;
    ss >> task_num;  // number of tasks
    //tasks_total.resize(task_num);
    this->num_of_tasks = task_num;
    for (int i = 0; i < task_num; i++)
    {
        int t,s, g, ts, tg;
        getline(myfile, line);
        ss.clear();
        ss << line;
        ss >> t >> s >> g >> ts >> tg; //time+start+goal+time at start+time at goal
        s=s% ml.endpoints.size();
        g=g% ml.endpoints.size();
        assert(s<ml.endpoints.size());
        assert(g<ml.endpoints.size());
        Task* new_task = new Task(i,t,ml.endpoints[s],ml.endpoints[g]);
        new_task->ideal_end_time = ml.getDistance(ml.endpoints[s],ml.endpoints[g]) + t;

        if(t > last_release_time){
            last_release_time = t;
        }
        all_tasks.push(new_task);
        all_tasks_vec.push_back(new_task);

    }
    myfile.close();
}
