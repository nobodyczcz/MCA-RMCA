/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, Dec 2018
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/

#include "map_loader.h"
#include "agents_loader.h"
#include "ICBSHSearchPairAnalysis.h"

#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

int main(int argc, char** argv) 
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("map,m", po::value<std::string>()->required(), "input file for map")
		("agents,a", po::value<std::string>()->required(), "input file for agents")
		("output,o", po::value<std::string>()->required(), "output file for schedule")
		("solver,s", po::value<std::string>()->required(), "solvers (CBS, ICBS, CBSH, CBSH-CR, CBSH-R, CBSH-RM, CBSH-GR")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<float>()->default_value(7200), "cutoff time (seconds)")
		("seed,d", po::value<int>()->default_value(0), "random seed")
		("screen", po::value<int>()->default_value(0), "screen option (0: none; 1: results; 2:all)")
		("cardinalRect", po::value<bool>(), "only consider cardinal rectangle conflicts")
		("corridor2", po::value<bool>(), "reason about 2-way branching corridor conflicts")
		("corridor4", po::value<bool>(), "reason about 4-way branching corridor conflicts")
		("cardinalCorridor", po::value<bool>(), "only reason about cardinal corridor conflicts")
		("target", po::value<bool>(), "reason about target conflict")
		("kDelay", po::value<int>()->default_value(0), "generate k-robust plan")
		("ignore-t0","Ignore t==0 k-delay conflict resolving")
		("asyConstraint","Using asymmetry range constraint to resolve k-delay conflict. Else use symmetry range constraint ")
		("short", "do not use long barrier constraint to resolve k delay rectangle conflict")
		("only_generate_instance", po::value<std::string>()->default_value(""),"no searching")
		("debug", "debug mode")
		("RM-4way", po::value<int>()->default_value(1), "0, do not do 4 way split. 1, do 4 way only necessary.2 Always do 4 way splitting for RM, other wise only 4 way when necessary")
		("flipped_rectangle", "resolving flipped rectangle symmetry conflict for RM")
		("I_RM","using improved rm")
		("statistic","print statistic data")
		("pairAnalysis",po::value<int>(),"perform 2 agent analysis")
		("printFailedPair","print mdd and constraints for failed pair")


            ;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);
	srand((int)time(0));

	if (vm["screen"].as<int>() == 2) {
		cout << "[CBS] Loading map and agents " << endl;
	}
	// read the map file and construct its two-dim array
	MapLoader* ml = new MapLoader(vm["map"].as<string>());

	// read agents' start and goal locations
	AgentsLoader al(vm["agents"].as<string>(), *ml, vm["agentNum"].as<int>());
	if (vm["screen"].as<int>() == 2) {
		cout << "[CBS] Loading map and agents don2! " << endl;
	}
	srand(vm["seed"].as<int>());

	options options1;
	if (vm.count("asyConstraint")) {
		options1.asymmetry_constraint = true;
	}
	else {
		options1.asymmetry_constraint = false;
	}

	if (vm.count("debug")) {
		options1.debug = true;
	}
	else {
		options1.debug = false;
	}

	if (vm.count("printFailedPair")){
	    options1.printFailedPair = true;
	}


	options1.RM4way = vm["RM-4way"].as<int>();

	if (vm.count("flipped_rectangle")) {
		options1.flippedRec = true;
	}
	else {
		options1.flippedRec = false;
	}

	if (vm.count("ignore-t0")) {
		options1.ignore_t0 = true;
	}
	else {
		options1.ignore_t0 = false;
	}
	if (vm.count("short")) {
		options1.shortBarrier = true;
	}
	else {
		options1.shortBarrier = false;
	}
    if (vm.count("pairAnalysis")) {
        options1.pairAnalysis = true;
    }


	if (vm["only_generate_instance"].as<string>()!="") {
		al.saveToFile(vm["only_generate_instance"].as<string>());
		return 0;
	}

	constraint_strategy s;
	if (vm["solver"].as<string>() == "ICBS")
		s = constraint_strategy::ICBS;
	else if (vm["solver"].as<string>() == "CBS")
		s = constraint_strategy::CBS;
	else if (vm["solver"].as<string>() == "CBSH")
		s = constraint_strategy::CBSH;
	else if (vm["solver"].as<string>() == "CBSH-CR")
		s = constraint_strategy::CBSH_CR;
	else if (vm["solver"].as<string>() == "CBSH-R")
		s = constraint_strategy::CBSH_R;
	else if (vm["solver"].as<string>() == "CBSH-RM")
		s = constraint_strategy::CBSH_RM;
	else if (vm["solver"].as<string>() == "CBSH-GR")
		s = constraint_strategy::CBSH_GR;
	else
	{
		std::cout <<"WRONG SOLVER NAME!" << std::endl;
		return -1;
	}

    ICBSSearchWithPairedAnalysis<MapLoader> icbs(ml, al, 1.0, s, vm["cutoffTime"].as<float>() * CLOCKS_PER_SEC, vm["screen"].as<int>(), vm["kDelay"].as<int>(), options1);
	if (vm["solver"].as<string>() == "CBSH-RM")
	{
		icbs.rectangleMDD = true;
	}
	if (vm.count("cardinalRect"))
	{
		icbs.cardinalRect = vm["cardinalRect"].as<bool>();
	}
	if (vm.count("corridor2"))
	{
		icbs.corridor2 = vm["corridor2"].as<bool>();
	}
	if (vm.count("corridor4"))
	{
		icbs.corridor4 = vm["corridor4"].as<bool>();
	}
	if (vm.count("cardinalCorridor"))
	{
		icbs.cardinalCorridorReasoning = vm["cardinalCorridor"].as<bool>();
	}
	if (vm.count("target"))
	{
		icbs.targetReasoning = vm["target"].as<bool>();

	}
	if (vm.count("I_RM"))
	{
		icbs.I_RM = true;

	}

    if (vm.count("pairAnalysis")) {
        icbs.analysisEngine = new ICBSSearchWithPairedAnalysis<MapLoader>(&icbs, vm["pairAnalysis"].as<int>());
        icbs.analysisOutput = ofstream();
        icbs.analysisOutput.open(vm["output"].as<string>()+".FailedPairs",ios::trunc);
        icbs.analysisOutput<<"["<<endl;
    }
	
	bool res;
	res = icbs.runICBSSearch();
	ofstream stats;
	stats.open(vm["output"].as<string>(), ios::trunc);
	stats << icbs.runtime/ CLOCKS_PER_SEC << "," <<
		icbs.HL_num_expanded << "," << icbs.HL_num_generated << "," <<
		icbs.LL_num_expanded << "," << icbs.LL_num_generated << "," <<
		vm["agents"].as<string>() << "," << icbs.solution_cost << "," << 
		icbs.min_f_val - icbs.dummy_start->g_val << "," <<
		vm["solver"].as<string>()  << "," <<
		icbs.num_standard << "," << icbs.num_rectangle << "," <<
		icbs.num_corridor2 << "," << icbs.num_corridor4 << "," << 
		icbs.num_target<<","<< icbs.num_chasingRectangle << "," <<
		icbs.less10 << ","<<icbs.less100 << ","<<icbs.less1000 << ","<<icbs.less10000 <<"," << icbs.less100000 <<","
		<<icbs.larger100000 << "," <<icbs.num_pairs <<"," <<icbs.num_failed << ","<< endl;
	stats.close();

    if(vm.count("statistic")){
        cout<<"Total RM time: "<<icbs.RMTime/CLOCKS_PER_SEC <<endl;
        cout<<"Total time on extract sg: " <<icbs.runtime_findsg/CLOCKS_PER_SEC <<endl;
        cout<<"Total time on find rectangle: " <<icbs.runtime_find_rectangle/CLOCKS_PER_SEC <<endl;
        cout<<"Total time on build mdd in RM: " <<icbs.RMBuildMDDTime/CLOCKS_PER_SEC <<endl;

        cout<<"Total RM detection: "<<icbs.RMDetectionCount <<endl;
        cout<<"Total RM find: "<<icbs.RMSuccessCount <<endl;
        cout<<"Total RM rejected before find rectangle: "<<icbs.RMFailBeforeRec <<endl;
        cout<<"Total MDD: "<<icbs.TotalMDD <<endl;
        cout<<"Total Exist MDD: "<<icbs.TotalExistMDD <<endl;
        cout<<"Total K MDD: "<<icbs.TotalKMDD <<endl;
        cout<<"Total Exist K MDD: "<<icbs.TotalExistKMDD <<endl;
        cout<<"Repeated pairs: "<<icbs.repeated_pairs<<endl;
        for (int i=0; i<icbs.KRectangleCount.size();i++){
            cout<< "total k: "<< i <<" rectangle: " << icbs.KRectangleCount[i]<<endl;
        }
    }

    if (vm.count("pairAnalysis")) {
        icbs.analysisOutput<<"]";
        icbs.analysisOutput.close();
    }

    if (vm["screen"].as<int>() == 2)
		cout << "Done!!" << endl;
	delete ml;
	return 0;

}
