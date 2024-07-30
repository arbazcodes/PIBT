#include "astar.h"
#include <iostream>

int main() {
    vector<vector<int>> grid = {
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1}
    };

    AStar astar(grid);

    Pair source = {1, 1};
    Pair destination = {1, 4};
    vector<tuple<int, int, int>> constraints = {{1, 2, 1}};

    vector<vector<int>> path = astar.aStarSearch(source, destination, constraints);

    for (const auto& node : path) {
        cout << "(" << node[0] << ", " << node[1] << ", " << node[2] << ") ";
    }
    cout << endl;

    return 0;
}
