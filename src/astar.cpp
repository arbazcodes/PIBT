#include "astar.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <stack>

AStar::AStar(const vector<vector<int>>& grid) : grid(grid), ROW(grid.size()), COL(grid[0].size()) {}

bool AStar::isValid(int row, int col) const {
    return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL);
}

bool AStar::isUnBlocked(int row, int col) const {
    return grid[row][col] == 1;
}

bool AStar::isDestination(int row, int col, Pair dest) const {
    return row == dest.second && col == dest.first;
}

double AStar::calculateHValue(int row, int col, Pair dest) const {
    return ((row - dest.second) + (col - dest.first));
}

bool AStar::violatesConstraints(int row, int col, int timestep, const vector<Constraint>& constraints) const {
    for (const auto& constraint : constraints) {
        int cx, cy, ct;
        cx = constraint.x;
        cy  = constraint.y;
        ct = constraint.time;
        if (cy == col && cx == row && ct == timestep) {
            return true;
        }
    }
    return false;
}

void AStar::initializeCells(vector<vector<Cell>>& cellDetails) {
    for (int i = 0; i < ROW; ++i) {
        for (int j = 0; j < COL; ++j) {
            cellDetails[i][j].f = numeric_limits<double>::max();
            cellDetails[i][j].g = numeric_limits<double>::max();
            cellDetails[i][j].h = numeric_limits<double>::max();
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
            cellDetails[i][j].time = -1;
        }
    }
}

vector<vector<int>> AStar::tracePath(const vector<vector<Cell>>& cellDetails, Pair dest) const {
    vector<vector<int>> path;
    stack<vector<int>> Path;
    int row = dest.second;
    int col = dest.first;

    while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col)) {
        Path.push({col, row, cellDetails[row][col].time});
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }
    Path.push({col, row, cellDetails[row][col].time});

    while (!Path.empty()) {
        path.push_back(Path.top());
        Path.pop();
    }

    return path;
}

vector<vector<int>> AStar::aStarSearch(Pair src, Pair dest, const vector<Constraint>& constraints) {
    vector<vector<int>> path;

    if (!isValid(src.second, src.first) || !isValid(dest.second, dest.first)) {
        cout << "Source or Destination is invalid\n";
        return path;
    }

    if (!isUnBlocked(src.second, src.first) || !isUnBlocked(dest.second, dest.first)) {
        cout << "Source or Destination is blocked\n";
        return path;
    }

    if (isDestination(src.second, src.first, dest)) {
        cout << "We are already at the destination\n";
        path.push_back({src.first, src.second});
        return path;
    }

    vector<vector<Cell>> cellDetails(ROW, vector<Cell>(COL));
    initializeCells(cellDetails);

    vector<vector<bool>> closedList(ROW, vector<bool>(COL, false));

    int i = src.second;
    int j = src.first;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;
    cellDetails[i][j].time = 0;

    priority_queue<pPair, vector<pPair>, greater<>> openList;
    openList.push({0.0, {i, j}});

    bool foundDest = false;

    vector<int> rowNum = {-1, 1, 0, 0};
    vector<int> colNum = {0, 0, -1, 1};

    while (!openList.empty()) {
        pPair p = openList.top();
        openList.pop();

        i = p.second.second;
        j = p.second.first;
        int timestep = cellDetails[i][j].time + 1;
        closedList[i][j] = true;

        for (int k = 0; k < 4; ++k) {
            int newRow = i + rowNum[k];
            int newCol = j + colNum[k];

            if (isValid(newRow, newCol) && !violatesConstraints(newCol, newRow, timestep, constraints)) {
                if (isDestination(newRow, newCol, dest)) {
                    cellDetails[newRow][newCol].parent_i = i;
                    cellDetails[newRow][newCol].parent_j = j;
                    cellDetails[newRow][newCol].time = timestep;
                    cout << "The destination cell is found\n";
                    path = tracePath(cellDetails, dest);
                    foundDest = true;
                    return path;
                } else if (!closedList[newRow][newCol] && isUnBlocked(newRow, newCol)) {
                    double gNew = cellDetails[i][j].g + 1.0;
                    double hNew = calculateHValue(newRow, newCol, dest);
                    double fNew = gNew + hNew;

                    if (cellDetails[newRow][newCol].f == numeric_limits<double>::max() || cellDetails[newRow][newCol].f > fNew) {
                        openList.push({fNew, {newCol, newRow}});
                        cellDetails[newRow][newCol].f = fNew;
                        cellDetails[newRow][newCol].g = gNew;
                        cellDetails[newRow][newCol].h = hNew;
                        cellDetails[newRow][newCol].parent_i = i;
                        cellDetails[newRow][newCol].parent_j = j;
                        cellDetails[newRow][newCol].time = timestep;
                    }
                }
            }
        }
    }

    if (!foundDest) {
        cout << "Failed to find the Destination Cell\n";
    }

    return path;
}

