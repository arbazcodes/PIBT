#ifndef CBS_H
#define CBS_H

#include <vector>
#include <utility>
#include "astar.h"
#include <queue>

using namespace std;

typedef vector<vector<int>> CostPath;

struct CBSNode {
    vector<CostPath> solution;
    vector<Constraint> constraints;
    int cost;

    bool operator<(const CBSNode& other) const {
        return cost > other.cost;
    }
};

class cbs
{
public:
    vector<vector<int>> Grid;

    cbs(vector<vector<int>> grid);
    int findTotalCost(vector<CostPath> Solution);
    vector<vector<int>> findConflicts(vector<CostPath> Solution);
    vector<Constraint> generateConstraints(vector<vector<int>> Conflicts);
    vector<CostPath> low_level(vector<Pair> sources, vector<Pair> destinatoins, const vector<Constraint>& constraints);
    vector<CostPath> high_level(vector<Pair> sources, vector<Pair> destinations);
};

 #endif