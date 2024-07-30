#ifndef CBS_H
#define CBS_H

#include <vector>
#include <utility>
#include "astar.h"

using namespace std;

typedef pair<int, int> Pair;


class cbs
{

public:
    
    AStar astar;

    cbs(vector<vector<int>> grid);
    vector<vector<int>> findConflicts(vector<vector<int>> Paths);
    vector<Constraint> generateConstraints(vector<vector<int>> Conflicts);
    vector<vector<vector<int>>> low_level(vector<Pair> sources, vector<Pair> destinatoins, const vector<Constraint>& constraints);
    vector<vector<vector<int>>> high_level(vector<Pair> sources, vector<Pair> destinatoins);
    
};





 #endif