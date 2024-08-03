#include "astar.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include <map>
#include <set>
#include <iostream>

// Function to calculate Manhattan distance
int ManhattanDistance(const Pair &a, const Pair &b)
{
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

// Function to calculate the cost of rotation between two directions
int RotationCost(Direction from, Direction to)
{
    return (from == to) ? 0 : 1;
}

// A* algorithm implementation
std::vector<std::vector<int>> AStarAlgorithm(
    const Pair &start,
    const Pair &goal,
    const std::vector<Constraint> &constraints,
    const std::vector<std::vector<int>> &grid)
{
    int rows = grid.size();
    int cols = (rows > 0) ? grid[0].size() : 0;

    std::vector<Pair> direction_vectors = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {0, 0}};
    std::vector<Direction> directions = {RIGHT, DOWN, LEFT, UP, STAY};

    std::priority_queue<
        std::tuple<int, State>,
        std::vector<std::tuple<int, State>>,
        std::greater<std::tuple<int, State>>>
        open_list;
    open_list.push({0, {start, RIGHT, 0}});

    std::map<State, int> g_costs;
    g_costs[{start, RIGHT, 0}] = 0;

    std::map<State, State> came_from;

    std::map<int, std::set<Pair>> vertex_constraint_map;
    std::map<int, std::set<std::pair<Pair, Pair>>> edge_constraint_map;

    for (const auto &constraint : constraints)
    {
        vertex_constraint_map[constraint.time].insert({constraint.x, constraint.y});
        // Edge constraint: store as a pair of positions
        if (constraint.id == -1) // Assuming an ID of -1 signifies edge constraints
        {
            edge_constraint_map[constraint.time].insert({{constraint.x, constraint.y}, {constraint.x + direction_vectors[RIGHT].first, constraint.y + direction_vectors[RIGHT].second}});
        }
    }

    while (!open_list.empty())
    {
        auto [_, current] = open_list.top();
        open_list.pop();

        if (current.position == goal)
        {
            std::vector<std::vector<int>> path;
            while (came_from.find(current) != came_from.end())
            {
                path.push_back({current.position.first, current.position.second, static_cast<int>(current.direction), current.time_step});
                current = came_from[current];
            }
            path.push_back({start.first, start.second, static_cast<int>(RIGHT), 0});
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < directions.size(); ++i)
        {
            Direction new_direction = static_cast<Direction>(i);
            Pair direction_vector = direction_vectors[i];
            Pair next_cell = {current.position.first + direction_vector.first, current.position.second + direction_vector.second};
            int next_time_step = current.time_step + 1;

            if (next_cell.first < 0 || next_cell.first >= rows || next_cell.second < 0 || next_cell.second >= cols || grid[next_cell.first][next_cell.second] != 1)
            {
                continue;
            }

            if (vertex_constraint_map.find(next_time_step) != vertex_constraint_map.end() && vertex_constraint_map[next_time_step].find(next_cell) != vertex_constraint_map[next_time_step].end())
            {
                continue;
            }

            bool edge_conflict = false;
            if (edge_constraint_map.find(next_time_step) != edge_constraint_map.end())
            {
                auto edge_constraints = edge_constraint_map[next_time_step];
                for (const auto &edge : edge_constraints)
                {
                    if ((edge.first == current.position && edge.second == next_cell) ||
                        (edge.second == current.position && edge.first == next_cell))
                    {
                        edge_conflict = true;
                        break;
                    }
                }
            }

            if (edge_conflict)
            {
                continue;
            }

            int rotation_cost_value = RotationCost(current.direction, new_direction);
            int move_cost = 1;
            int final_g_cost = g_costs[current] + rotation_cost_value + move_cost;
            State final_state = {next_cell, new_direction, next_time_step};

            if (g_costs.find(final_state) == g_costs.end() || final_g_cost < g_costs[final_state])
            {
                g_costs[final_state] = final_g_cost;
                int f_cost = final_g_cost + ManhattanDistance(final_state.position, goal);
                open_list.push({f_cost, final_state});
                came_from[final_state] = current;
            }
        }
    }

    return {};
}
