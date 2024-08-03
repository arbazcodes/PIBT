#include "cbs.h"
#include <map>
#include <iostream>
#include <optional>
#include <queue>
#include <set>

// Constructor
Cbs::Cbs(const std::vector<std::vector<int>> &grid) : grid(grid) {}

std::optional<std::vector<CostPath>> Cbs::LowLevel(
    const std::vector<Pair> &sources,
    const std::vector<Pair> &destinations,
    const std::vector<Constraint> &constraints) const
{
    std::vector<CostPath> solution;
    std::map<int, std::vector<Constraint>> constraint_by_id;

    for (const auto &constraint : constraints)
    {
        constraint_by_id[constraint.id].push_back(constraint);
    }

    for (int i = 0; i < sources.size(); ++i)
    {
        auto path = AStarAlgorithm(sources[i], destinations[i], constraint_by_id[i], grid);

        if (path.empty())
        {
            //std::cout << "No path found for agent " << i << " with constraints." << std::endl;
            return std::nullopt;
        }
        solution.push_back(path);
    }

    return solution;
}

// Calculate the total cost of a solution
int Cbs::FindTotalCost(const std::vector<CostPath> &solution) const
{
    int total_cost = 0;

    for (const auto &path : solution)
    {
        total_cost += path.size();
    }

    return total_cost;
}

std::vector<std::vector<int>> Cbs::FindConflicts(const std::vector<CostPath> &solution) const
{
    std::vector<std::vector<int>> conflicts;

    // Check for vertex conflicts
    auto vertex_conflicts = FindConflictsVertex(solution);
    conflicts.insert(conflicts.end(), vertex_conflicts.begin(), vertex_conflicts.end());

    // Check for edge conflicts
    auto edge_conflicts = FindConflictsEdge(solution);
    conflicts.insert(conflicts.end(), edge_conflicts.begin(), edge_conflicts.end());

    return conflicts;
}

std::vector<std::vector<int>> Cbs::FindConflictsVertex(const std::vector<CostPath> &solution) const
{
    std::vector<std::vector<int>> vertex_conflicts;

    std::map<Pair, std::set<int>> cell_occupancy; // Mapping from cell to set of agents occupying it

    for (int i = 0; i < solution.size(); ++i)
    {
        const auto &path = solution[i];

        for (int t = 0; t < path.size(); ++t)
        {
            const auto &step = path[t];
            Pair cell = {step[0], step[1]};

            if (cell_occupancy[cell].find(i) != cell_occupancy[cell].end())
            {
                vertex_conflicts.push_back({i, i, step[0], step[1], t});
            }
            else
            {
                cell_occupancy[cell].insert(i);
            }
        }
    }

    return vertex_conflicts;
}

std::vector<std::vector<int>> Cbs::FindConflictsEdge(const std::vector<CostPath> &solution) const
{
    std::vector<std::vector<int>> Conflicts;

    for (int i = 0; i < solution.size(); ++i)
    {
        const std::vector<std::vector<int>> &path_1 = solution[i];

        for (int j = i + 1; j < solution.size(); ++j)
        {
            const std::vector<std::vector<int>> &path_2 = solution[j];

            // Check for edge conflicts
            for (int t = 0; t < path_1.size() && t < path_2.size(); ++t)
            {
                const auto &step_1 = path_1[t];
                const auto &step_2 = path_2[t];

                if (step_1.size() == 4 && step_2.size() == 4)
                {
                    int x1 = step_1[0];
                    int y1 = step_1[1];
                    int x2 = step_2[0];
                    int y2 = step_2[1];

                    if ((x1 == x2 && y1 == y2) ||               
                        (x1 == x2 && y1 == y2 && step_1[2] != step_2[2]))
                    {
                        Conflicts.push_back({i, j, x1, y1, t});
                    }
                }
            }
        }
    }

    return Conflicts;
}

std::vector<Constraint> Cbs::GenerateConstraints(const std::vector<std::vector<int>> &conflicts) const
{
    std::vector<Constraint> Constraints;

    for (const auto &p : conflicts)
    {
        {
            Constraints.push_back({p[0], p[2], p[3], p[4]});
            Constraints.push_back({p[1], p[2], p[3], p[4]});
        }
    }

    return Constraints;
}

std::vector<CostPath> Cbs::HighLevel(const std::vector<Pair> &sources, const std::vector<Pair> &destinations) const
{
    std::priority_queue<CbsNode> open;

    CbsNode root;
    root.constraints = {};

    auto initial_solution = LowLevel(sources, destinations, root.constraints);
    if (!initial_solution)
    {
        std::cout << "No initial solution found." << std::endl;
        return {};
    }

    root.solution = *initial_solution;
    root.cost = FindTotalCost(root.solution);
    open.push(root);

    while (!open.empty())
    {
        CbsNode current = open.top();
        open.pop();

        auto conflicts = FindConflicts(current.solution);

        if (conflicts.empty())
        {
            std::cout << "Solution found with total cost: " << current.cost << std::endl;
            return current.solution;
        }

        auto conflict = conflicts[0];

        std::vector<Constraint> new_constraints = GenerateConstraints({conflict});

        for (int i = 0; i < 2; ++i)
        {
            CbsNode child = current;
            child.constraints.push_back(new_constraints[i]);

            auto new_solution = LowLevel(sources, destinations, child.constraints);

            if (!new_solution.has_value())
                continue;

            child.solution = new_solution.value();
            child.cost = FindTotalCost(child.solution);
            open.push(child);
        }
    }
    std::cout << "No feasible solution found." << std::endl;
    return {};
}
