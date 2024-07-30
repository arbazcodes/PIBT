#include "astar.h"
#include <iostream>
#include <vector>

using namespace std;

int main() {

    AStar astar("C:/Users/Lenovo/Desktop/Sim/levels/one.lvl");

    vector<Pair> sources = {{1, 1}, {2, 2}}; 

    // Find paths using A* search
    vector<vector<int>> path = astar.aStarSearch(make_pair(1, 1), make_pair(4, 4), {2, 1, 2});

    for (const auto& node : path) 
    {
        cout << "(" << node[0] << ", " << node[1] << ", "<<node[2]<<") ";
    }
    cout << endl;

    return 0;
}
