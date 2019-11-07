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

    // assign priority ordering to agents
    // By default, we use the index ordering of the agents where
    // the first always has the highest priority.
    list<int> priorities;
    // For making 0 highest priority
    // for (int i = 0; i < ins.num_of_agents; i++) {
    //     priorities.push_back(i);
    // }
    // For making 1 highest priority
    for (int i = ins.num_of_agents-1; i >= 0; i--) {
        priorities.push_back(i);
    }

    list<Constraint> constraints;

    // plan paths
    int longestPath = 0;
    int previous = -1;
    for (int i : priorities) {
        // TODO: Transform already planned paths into constraints
        if (previous != -1) {
            int size = paths[previous].size();
            for (int j=0; j<size-1; j++) {
                int x = paths[previous][j];
                int y = paths[previous][j+1];
                // vertex collision
                for (auto next_location : ins.get_adjacent_locations(y)) {
                    if (next_location != y) {
                        constraints.push_back(make_tuple(i, next_location, y, j+1));
                    }
                }
                // edge collision
                constraints.push_back(make_tuple(i,y,x,j+1));
                constraints.push_back(make_tuple(i,y,-1,j+1));

            }
            constraints.push_back(make_tuple(previous,paths[previous][size-1],-2,size-1));
        }

        //  Replace the following line with something like paths[i] = a_star.find_path(i, constraints);  
        paths[i] = a_star.find_path(i, constraints,longestPath);
        previous = i;
        if (paths[i].size() > longestPath) {
            longestPath = paths[i].size();
        }
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