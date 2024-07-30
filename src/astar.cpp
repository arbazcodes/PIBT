#include "astar.h"
#include <queue>
#include <cmath>
#include <algorithm>

namespace std {
    // Hash function for pair of Pair and int
    template <>
    struct hash<pair<Pair, int>> {
        size_t operator()(const pair<Pair, int>& p) const {
            return hash<int>()(p.first.first) ^ hash<int>()(p.first.second) ^ hash<int>()(p.second);
        }
    };

    // Hash function for Pair
    template <>
    struct hash<Pair> {
        size_t operator()(const Pair& p) const {
            return hash<int>()(p.first) ^ hash<int>()(p.second);
        }
    };
}

int manhattan_distance(const Pair& a, const Pair& b) {
    return abs(a.first - b.first) + abs(a.second - b.second);
}

vector<vector<int>> a_star_algorithm(const Pair& start, const Pair& goal, const vector<Constraint>& constraints, const vector<vector<int>>& grid) {
    int rows = grid.size();
    int cols = rows > 0 ? grid[0].size() : 0;

    // Directions for moving in the grid (right, down, left, up)
    vector<Pair> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

    // Priority queue for A* search
    priority_queue<tuple<int, Pair, int>, vector<tuple<int, Pair, int>>, greater<tuple<int, Pair, int>>> open_list;
    open_list.push({0, start, 0});  // (priority, (x, y), time_step)

    // Distance cost map
    unordered_map<Pair, int, hash<Pair>> g_costs;
    g_costs[start] = 0;

    // Map for storing the path
    unordered_map<pair<Pair, int>, pair<Pair, int>, hash<pair<Pair, int>>> came_from;

    // Set of constraints for quick lookup
    unordered_map<int, unordered_set<Pair>> constraint_map;
    for (const auto& c : constraints) {
        constraint_map[c.time].insert({c.x, c.y});
    }

    while (!open_list.empty()) {
        auto [_, current, time_step] = open_list.top();
        open_list.pop();

        if (current == goal) {
            vector<vector<int>> path;
            while (came_from.find({current, time_step}) != came_from.end()) {
                path.push_back({current.first, current.second, time_step});
                auto [prev, prev_time_step] = came_from[{current, time_step}];
                current = prev;
                time_step = prev_time_step;
            }
            path.push_back({start.first, start.second, 0});
            reverse(path.begin(), path.end());
            return path;
        }

        // Explore neighbors
        for (const auto& direction : directions) {
            Pair next_cell = {current.first + direction.first, current.second + direction.second};
            int next_time_step = time_step + 1;

            // Check if next cell is within bounds and not blocked
            if (next_cell.first < 0 || next_cell.first >= rows ||
                next_cell.second < 0 || next_cell.second >= cols ||
                grid[next_cell.first][next_cell.second] != 1) {
                continue;
            }

            // Check constraints
            if (constraint_map.find(next_time_step) != constraint_map.end() &&
                constraint_map[next_time_step].find(next_cell) != constraint_map[next_time_step].end()) {
                continue;
            }

            // Calculate the cost
            int new_g_cost = g_costs[current] + 1; // Assuming each move has cost 1

            if (g_costs.find(next_cell) == g_costs.end() || new_g_cost < g_costs[next_cell]) {
                g_costs[next_cell] = new_g_cost;
                int f_cost = new_g_cost + manhattan_distance(next_cell, goal);
                open_list.push({f_cost, next_cell, next_time_step});
                came_from[{next_cell, next_time_step}] = {current, time_step};
            }
        }
    }

    return {}; // If no path is found
}
