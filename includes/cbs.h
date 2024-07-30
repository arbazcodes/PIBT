#ifndef CBS_H
#define CBS_H

#include <vector>
#include <utility>

using namespace std;

typedef pair<int, int> Pair;

class cbs
{

public:
    cbs();

    vector<vector<int>> findConflicts(vector<vector<int>> Paths);

    vector<vector<int>> generateConstraints(vector<vector<int>> Conflicts);
    
    vector<vector<vector<int>>> cbs_low_level(vector<Pair> sources, vector<Pair> destinatoins);
    vector<vector<vector<int>>> cbs_high_level(vector<Pair> sources, vector<Pair> destinatoins);
    
};





 #endif