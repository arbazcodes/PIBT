#include <iostream>
#include "cbs.h"

int main() {
    vector<Pair> starts = {{1, 2}, {2, 0}, {0, 1}};
    vector<Pair> goals = {{2, 1}, {2, 3}, {4, 1}};
    vector<Constraint> constraints = {
        // {0, 3, 2, 1}, {1, 0, 3, 1}, {1, 3, 4, 5}
    };
    vector<vector<int>> grid = {
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1}
    };

    cbs cbs(grid);

    vector<CostPath> Solution = cbs.low_level(starts, goals, constraints);

    for (size_t i = 0; i < starts.size(); i++)
    {
        cout <<"Cost: "<<Solution[i].first<< "  Path: ";
        for (const auto& p : Solution[i].second) {
            cout << "(" << p[0] << ", " << p[1] << ", " << p[2] << ") ";
        }
        cout << endl;
    }

    vector<vector<int>> Conflicts = cbs.findConflicts(Solution);

    cout <<"Conflicts: ";
    for (const auto& p : Conflicts) {
        cout << "(" << p[0] << ", " << p[1] << ", " << p[2] <<", "<< p[3]<< ", "<< p[4]<<") ";
    }
    cout << endl;

    vector<Constraint> Constraints = cbs.generateConstraints(Conflicts);

    cout <<"Constraints: ";
    for (const auto& p : Constraints) {
        cout << "(" << p.ID << ", " << p.x << ", " << p.y <<", "<< p.time<<") ";
    }
    cout << endl;

    return 0;
}
