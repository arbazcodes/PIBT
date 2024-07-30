#include "cbs.h"
#include <map>
#include <unordered_map>
#include <unordered_set>

cbs::cbs(vector<vector<int>> grid): Grid(grid){}

vector<CostPath> cbs::low_level(vector<Pair> sources, vector<Pair> destinatoins, const vector<Constraint>& constraints)
{

    vector<CostPath> Solution;

    for (size_t i = 0; i < sources.size(); i++)
    {
        map<int, vector<Constraint>> constraintByID;
        for (const auto& constraint : constraints) {
            constraintByID[constraint.ID].push_back(constraint);
        }
        vector<vector<int>> path = a_star_algorithm(sources[i], destinatoins[i], constraintByID[i], Grid);
        Solution.push_back(make_pair(path.size(), path));
    }

    return Solution;
}

vector<vector<int>> cbs::findConflicts(vector<CostPath> Solution) {
    vector<vector<int>> Conflicts;

    for (int i = 0; i < Solution.size(); ++i) {
        const vector<vector<int>>& path_1 = Solution[i].second;
        
        for (int j = i + 1; j < Solution.size(); ++j) {
            const vector<vector<int>>& path_2 = Solution[j].second;

            for (int t = 0; t < path_1.size() && t < path_2.size(); ++t) {
                const auto& step_1 = path_1[t];
                const auto& step_2 = path_2[t];
                
                if (step_1.size() == 3 && step_2.size() == 3) {
                    int x1 = step_1[0];
                    int y1 = step_1[1];
                    int x2 = step_2[0];
                    int y2 = step_2[1];

                    if (x1 == x2 && y1 == y2) {
                        Conflicts.push_back({i, j, x1, y1, static_cast<int>(t)});
                    }
                }
            }
        }
    }

    return Conflicts;
}

vector<Constraint> cbs::generateConstraints(vector<vector<int>> Conflicts)
{
    vector<Constraint> Constraints;

    for (const auto& p : Conflicts) {
        Constraints.push_back({p[0], p[2], p[3], p[4]});
        Constraints.push_back({p[1], p[2], p[3], p[4]});
    }

    return Constraints;
}

