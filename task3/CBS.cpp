#include "CBS.h"
#include <iostream>
#include <queue>

vector<Path> CBS::find_solution() {
    priority_queue<CBSNode*, vector<CBSNode*>, CompareCBSNode> open; // open list

    /* generate the root CBS node */
    auto root = new CBSNode();
    all_nodes.push_back(root);  // whenever generating a new node, we need to
                                 // put it into all_nodes
                                 // so that we can release the memory properly later in ~CBS()

    // find paths for the root node
    int numAgents = a_star.ins.num_of_agents;
    root->paths.resize(a_star.ins.num_of_agents);
    for (int i = 0; i < a_star.ins.num_of_agents; i++) {
        // TODO: if you change the input format of function find_path()
        //  you also need to change the following line to something like
        //  root->paths[i] = a_star.find_path(i, list<Constraint>());
        root->paths[i] = a_star.find_path(i, list<Constraint>(),0);
        if (root->paths[i].empty()) {
            cout << "Fail to find a path for agent " << i << endl;
            return vector<Path>(); // return "No solution"
        }
    }
    // compute the cost of the root node
    for (const auto& path : root->paths)
        root->cost += (int)path.size() - 1;

    // put the root node into open list
    open.push(root);

    while (!open.empty()) {

        // TODO: implement the high-level of CBS
        //get lowest cost node
        auto current = open.top();
        open.pop();


        // is collision free?
        bool collisionExists = false;
        int positions[current->cost][numAgents];
        list<Constraint> constraints;

        //populate positiosn array
        for (int agent=0; agent<numAgents; agent++) {
            Path path = current->paths[agent];
            int goalstate;
            for (int time=0; time<current->cost; time++) {
                if (time >= path.size()) {
                    positions[time][agent] = goalstate;
                } else {
                    positions[time][agent] = path[time];
                    goalstate = path[time];
                }
            }
        }

        // // print
        // cout << endl;
        // for (int agent=0; agent<numAgents;agent++) {
        //     for (int cost=0; cost<current->cost; cost++) {
        //         cout << positions[cost][agent] << " ";
        //     }
        //     cout << endl;
        // }
        // cout << "----------------";

        for (int time=0; time<current->cost; time++){
            // cout << "time: " << time << endl;
            //loop through agents
            for (int agent=0; agent<numAgents; agent++) {
                for (int i=0;i<numAgents;i++) {
                    if (i!=agent) {
                        //vertex collision
                        if (positions[time][i] == positions[time][agent]) {
                            collisionExists = true;
                            int collision = positions[time][i];
                            //impose constraint on a
                            constraints.remove(make_tuple(agent,collision,-1,time));
                            constraints.push_back(make_tuple(agent,collision,-1,time));
                            //impose constraint on b
                            constraints.remove(make_tuple(i,collision,-1,time));
                            constraints.push_back(make_tuple(i,collision,-1,time));
                            // cout << "\tvertex collision at " << time << ": ("<< positions[time][i] << ", " << positions[time][agent] << ")\n"; 
                            break;
                        }

                        //edge collision
                        if (time>0) {
                            int a = positions[time-1][agent];
                            int b = positions[time][agent];
                            int c = positions[time-1][i];
                            int d = positions[time][i];
                            if (a==d && b==c) {
                                collisionExists=true;
                                //impose constraint on a
                                constraints.remove(make_tuple(agent,a,b,time));
                                constraints.push_back(make_tuple(agent,a,b,time));
                                //impose constraint on b
                                constraints.remove(make_tuple(i,b,a,time));
                                constraints.push_back(make_tuple(i,b,a,time));
                                // cout << "\tedge collision at (" << positions[time][i] << ", " << time << ")\n"; 
                                break;
                            }
                        }
                    } 
                }
                if (!collisionExists) {
                    break;
                }
            }
        }//end collisions
        if (!collisionExists) {
            return current->paths;
        }


        for (auto constraint : constraints) {
            auto q = new CBSNode();
            all_nodes.push_back(q);
            q->constraints = current->constraints;
            bool duplicate = false;
            for (auto c : q-> constraints) {
                if (c == constraint) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) {continue;}
            // cout << "Adding constraint node: " << get<1>(constraint)<<endl;
            q->constraints.push_back(constraint);
            q->paths = current->paths;
            int a = get<0>(constraint);


            Path path = a_star.find_path(a, q->constraints,current->cost);
            // cout << "The new path is: \n";
            // for (auto pos: path) {
            //     cout << " " << pos;
            // }
            // cout << endl;

            //check whether to add this new path 
            if (!(path.empty())) {
                q->paths[a] = path;
                //compute new cost
                for (const auto& path : q->paths) {
                    q->cost += (int)path.size() - 1;
                }
                open.push(q);

                // cout <<"This new node has paths:";
                // for (auto path: q->paths) {
                //     for (int pos : path) {
                //         cout << " " << pos;
                //     }
                //     cout << endl;
                // }
                // cout << endl;
                // cout <<"This new node has constraints:";
                // for (auto cons: q->constraints) {
                //     cout << "(" << get<0>(cons);
                //     cout << ", " << get<1>(cons);
                //     cout << ", " << get<2>(cons);
                //     cout << ", " << get<3>(cons) << ")\n";
                // }
                // cout << endl;
            }
        }
    }

    cout << "No solution" << endl;
    return vector<Path>(); // return "No solution"
}


CBS::~CBS() {
    // release the memory
    for (auto n : all_nodes)
        delete n;
}