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

struct Constraint
{
    int ID, x, y, time;
};

class AStar {
public:
    AStar();
    AStar(const vector<vector<int>>& grid);
    vector<vector<int>> aStarSearch(Pair src, Pair dest, const vector<Constraint>& constraints);

private:
    vector<vector<int>> grid;
    int ROW, COL;

    void initializeCells(vector<vector<Cell>>& cellDetails);
    bool isValid(int row, int col) const;
    bool isUnBlocked(int row, int col) const;
    bool isDestination(int row, int col, Pair dest) const;
    double calculateHValue(int row, int col, Pair dest) const;
    vector<vector<int>> tracePath(const vector<vector<Cell>>& cellDetails, Pair dest) const;
    bool violatesConstraints(int row, int col, int timestep, const vector<Constraint>& constraints) const;
};

#endif
