#include "cbs.h"

#include <map>

cbs::cbs(vector<vector<int>> grid): astar(grid){}

vector<vector<vector<int>>> cbs::low_level(vector<Pair> sources, vector<Pair> destinatoins, const vector<Constraint>& constraints)
{

    vector<vector<vector<int>>> Paths;

    for (size_t i = 0; i < sources.size(); i++)
    {
        map<int, vector<Constraint>> constraintByID;
        for (const auto& constraint : constraints) {
            constraintByID[constraint.ID].push_back(constraint);
        }

        Paths.push_back(astar.aStarSearch(sources[i], destinatoins[i], constraintByID[i]));
    }

    return Paths;
    
}
