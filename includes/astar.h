#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <utility>

using namespace std;

typedef pair<int, int> Pair;  // (row, col)
typedef pair<double, Pair> pPair;

struct Cell {
    double f, g, h;
    int parent_i, parent_j;
    int time;
};

class AStar {
public:
    AStar(const vector<vector<int>>& grid);
    vector<vector<int>> aStarSearch(Pair src, Pair dest, const vector<tuple<int, int, int>>& constraints);

private:
    vector<vector<int>> grid;
    int ROW, COL;

    void initializeCells(vector<vector<Cell>>& cellDetails);
    bool isValid(int row, int col) const;
    bool isUnBlocked(int row, int col) const;
    bool isDestination(int row, int col, Pair dest) const;
    double calculateHValue(int row, int col, Pair dest) const;
    vector<vector<int>> tracePath(const vector<vector<Cell>>& cellDetails, Pair dest) const;
    bool violatesConstraints(int row, int col, int timestep, const vector<tuple<int, int, int>>& constraints) const;
};

#endif
