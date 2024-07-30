#ifndef CBS_H
#define CBS_H

#include <vector>
#include <utility>
#include "astar.h"

using namespace std;

typedef pair<int, vector<vector<int>>> CostPath;

class cbs
{

public:

    vector<vector<int>> Grid;
    cbs(vector<vector<int>> grid);
    vector<vector<int>> findConflicts(vector<CostPath> Solution);
    vector<Constraint> generateConstraints(vector<vector<int>> Conflicts);
    vector<CostPath> low_level(vector<Pair> sources, vector<Pair> destinatoins, const vector<Constraint>& constraints);
    vector<vector<vector<int>>> high_level(vector<Pair> sources, vector<Pair> destinatoins);
    
};
 #endif