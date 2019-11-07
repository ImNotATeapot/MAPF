#include <iostream>
#include <fstream>
#include "MAPFInstance.h"
#include "AStarPlanner.h"
#include <tuple>

int main(int argc, char *argv[]) {
    MAPFInstance ins;
    string input_file = argv[1];
    string output_file = argv[2];
    if (ins.load_instance(input_file)) {
        ins.print_instance();
    } else {
        cout << "Fail to load the instance " << input_file << endl;
        exit(-1);
    }

    AStarPlanner a_star(ins);
    vector<Path> paths(ins.num_of_agents);
    for (int i = 0; i < ins.num_of_agents; i++) {
        list<Constraint> constraints;
        // TODO: Define constraints
        //  constraints for Q1
        	// constraints.push_back(make_tuple(0,ins.goal_locations[0],-1,4));
	        // for (auto next_location : ins.get_adjacent_locations(ins.start_locations[1])) {
	        // 	if (next_location != ins.start_locations[1]) {
	        // 		constraints.push_back(make_tuple(1, ins.start_locations[1], next_location, 1));
	        // 	}
	        // }
						//Resulting paths:
						// 	        Paths:
						// 			a0: 8 9 10 11 11 12 
						//			a1: 9 9 10 11 


        //  constraints for Q2
	        // constraints.push_back(make_tuple(0,ins.goal_locations[0],-1,10));
						//Resulting Paths:
						// 	        Paths:
						// a0: 8 9 10 11 12 12 12 12 12 11 11 12 12 12 12 
						// a1: 9 10 11 11 11 11 11 11 11 11 11 11 11 11 11 


        //  constraints for Q3
        	constraints.push_back(make_tuple(1,10,-1,2));
        	constraints.push_back(make_tuple(1,8,-1,2));
        	constraints.push_back(make_tuple(1,9,-1,2));
        	constraints.push_back(make_tuple(1,11,-1,2));
        	constraints.push_back(make_tuple(1,12,-1,2));
        	constraints.push_back(make_tuple(0,12,-1,3));
        	
						//Resulting Paths:
						//         	Paths:
						// a0: 8 9 10 11 12 
						// a1: 9 10 17 10 11 
						// Sum of cost: 8

        //  Replace the following line with something like paths[i] = a_star.find_path(i, constraints);
        paths[i] = a_star.find_path(i, constraints);

        if (paths[i].empty()) {
            cout << "Fail to find any solutions for agent " << i << endl;
            return 0;
        }
    }

    // print paths
    cout << "Paths:" << endl;
    int sum = 0;
    for (int i = 0; i < ins.num_of_agents; i++) {
        cout << "a" << i << ": " << paths[i] << endl;
        sum += (int)paths[i].size() - 1;
    }
    cout << "Sum of cost: " << sum << endl;

    // save paths
    ofstream myfile (output_file.c_str(), ios_base::out);
    if (myfile.is_open()) {
        for (int i = 0; i < ins.num_of_agents; i++) {
            myfile << paths[i] << endl;
        }
        myfile.close();
    } else {
        cout << "Fail to save the paths to " << output_file << endl;
        exit(-1);
    }

    return 0;
}