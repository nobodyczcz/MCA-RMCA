//
// Created by Zhe Chen on 10/6/20.
//

#include "Agent.h"

Agent::Agent(int id, int initial_loc, int capacity) {
    this->agent_id = id;
    this->initial_location = initial_loc;
    this->capacity = capacity;
}

AgentLoader::AgentLoader(const std::string fname, const MapLoaderCost &ml){
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
            // read start [row,col] for agent i
            int row = atoi ( (*c_beg).c_str() );
            c_beg++;
            int col = atoi ( (*c_beg).c_str() );
            int loc = row*ml.cols + col;
            c_beg++;
            int capacity = atoi ((*c_beg).c_str());
            agents.push_back(new Agent(i,loc,capacity));
        }
        myfile.close();
    }
}

void AgentLoader::loadKiva(const std::string fname,int capacity, const MapLoaderCost &ml) {
    string line;
    ifstream myfile (fname.c_str());
    if (myfile.is_open()) {
        getline(myfile, line);
        boost::char_separator<char> sep(",");
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
        boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
        int rows = atoi((*beg).c_str()) + 2; // read number of rows
        beg++;
        int cols = atoi((*beg).c_str()) + 2; // read number of cols

        stringstream ss;
        getline(myfile, line);
        ss << line;

        ss.clear();
        getline(myfile, line);
        ss << line;

        ss.clear();
        getline(myfile, line);
        ss << line;
        ss.clear();

        //DeliverGoal.resize(row*col, false);
        // read map
        // read map (and start/goal locations)
        num_of_agents = 0;
        for (int i=1; i<rows-1; i++) {
            getline (myfile, line);
            for (int j=1; j<cols-1; j++) {
                if (line[j - 1] == 'r') //robot rest
                {
                    agents.push_back(new Agent(num_of_agents,i*ml.cols + j,capacity));
                    num_of_agents++;
                }
            }
        }

        myfile.close();
    }
    else
    {
        cerr << "file " << fname << " not found." << std::endl;
        exit(10);
    }
}