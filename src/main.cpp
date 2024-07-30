#include "cbs.h"
#include <iostream>

int main() {
    vector<vector<int>> grid = {
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1}
    };

    cbs cbs(grid);

    vector<Pair> sources = {{1, 1}, {2, 2}};
    vector<Pair> destinations = {{1, 4}, {3, 3}};
    vector<Constraint> constraints = {{0, 1, 2, 1}, {1, 3, 3, 2}, {1, 2, 3, 1}};

    vector<vector<vector<int>>> paths = cbs.low_level(sources, destinations, constraints);

    for (size_t i = 0; i < sources.size(); i++)
    {
        for (const auto& node : paths[i]) {
            cout << "(" << node[0] << ", " << node[1] << ", " << node[2] << ") ";
        }
        cout << endl;
    }
    

    return 0;
}
