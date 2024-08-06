#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <iterator>
#include <set>
#include "cbs.h"

// Define a type alias for Pair
using Pair = std::pair<int, int>;

// Function to generate unique random positions within grid bounds
std::vector<Pair> GenerateUniqueRandomPositions(int num_positions, int grid_rows, int grid_cols, std::mt19937 &rng)
{
    if (num_positions > grid_rows * grid_cols)
    {
        throw std::invalid_argument("Number of positions requested exceeds the number of available grid cells.");
    }

    std::vector<Pair> all_positions;
    for (int row = 0; row < grid_rows; ++row)
    {
        for (int col = 0; col < grid_cols; ++col)
        {
            all_positions.emplace_back(row, col);
        }
    }

    std::shuffle(all_positions.begin(), all_positions.end(), rng);
    all_positions.resize(num_positions);
    return all_positions;
}

// Function to ensure start and goal positions are different
void EnsureUniqueStartAndGoal(std::vector<Pair> &starts, std::vector<Pair> &goals, std::mt19937 &rng)
{
    std::set<Pair> start_set(starts.begin(), starts.end());
    std::set<Pair> goal_set(goals.begin(), goals.end());

    while (start_set.size() < starts.size() || goal_set.size() < goals.size())
    {
        if (start_set.size() < starts.size())
        {
            starts = GenerateUniqueRandomPositions(starts.size(), 3, 3, rng);
            start_set = std::set<Pair>(starts.begin(), starts.end());
        }
        if (goal_set.size() < goals.size())
        {
            goals = GenerateUniqueRandomPositions(goals.size(), 3, 3, rng);
            goal_set = std::set<Pair>(goals.begin(), goals.end());
        }
    }

    for (size_t i = 0; i < starts.size(); ++i)
    {
        while (start_set.find(goals[i]) != start_set.end())
        {
            // Generate a new goal position if it overlaps with a start position
            goals[i] = GenerateUniqueRandomPositions(1, 3, 3, rng)[0];
        }
    }
}

int main()
{
    const int num_agents = 6; // Number of agents
    const int grid_rows = 3;
    const int grid_cols = 3;
    const int num_iterations = 100; // Number of iterations to run

    // Initialize the grid
    std::vector<std::vector<int>> grid(grid_rows, std::vector<int>(grid_cols, 1));

    // Setup random number generation
    std::random_device rd;
    std::mt19937 rng(rd());

    for (int iteration = 0; iteration < num_iterations; ++iteration)
    {
        // Generate unique random sources and destinations
        auto starts = GenerateUniqueRandomPositions(num_agents, grid_rows, grid_cols, rng);
        auto goals = GenerateUniqueRandomPositions(num_agents, grid_rows, grid_cols, rng);

        // Ensure that starts and goals are different
        EnsureUniqueStartAndGoal(starts, goals, rng);

        // Define constraints (empty in this case)
        std::vector<Constraint> constraints = {};

        // Initialize the CBS algorithm with the grid
        Cbs cbs_algorithm(grid);

        for (int i = 0; i < starts.size(); i++)
        {
            std::cout << "Start: (" << starts[i].first << ", " << starts[i].second << ")---" << "Goal: (" << goals[i].first << ", " << goals[i].second << ")" << std::endl;
        }

        // Compute the solution using the high-level CBS algorithm
        std::vector<CostPath> solution = cbs_algorithm.HighLevel(starts, goals);

        if (solution.empty())
        {
            std::cout << "Iteration " << iteration << ": No valid solution found." << std::endl;
            continue; // Proceed to the next iteration
        }

        // Output the solution paths and costs
        std::cout << "Iteration " << iteration << ": Final Solution Paths and Costs:" << std::endl;
        for (size_t i = 0; i < starts.size(); ++i)
        {
            std::cout << "Agent " << i << " - Cost: " << solution[i].size() << " Path: ";
            int x = 0;
            for (const auto &step : solution[i])
            {
                std::cout << "(" << step[0] << ", " << step[1] << ", " << step[2] << ", " << step[3] << ")";
                if (!(x == solution[i].size() - 1))
                    std::cout << "--->";
                x++;
            }
            std::cout << std::endl;
        }

        // Calculate and output the total cost of the solution
        int total_cost = cbs_algorithm.FindTotalCost(solution);
        std::cout << "Iteration " << iteration << ": Total Cost: " << total_cost << std::endl;

        // Find and output conflicts in the solution
        std::vector<std::vector<int>> conflicts = cbs_algorithm.FindConflicts(solution);
        std::cout << "Iteration " << iteration << ": Conflicts: ";
        for (const auto &conflict : conflicts)
        {
            std::cout << "(" << conflict[0] << ", " << conflict[1] << ", " << conflict[2] << ", " << conflict[3] << ", " << conflict[4] << ") ";
        }
        std::cout << std::endl;

        // Generate and output new constraints based on conflicts
        std::vector<Constraint> new_constraints = cbs_algorithm.GenerateConstraints(conflicts);
        std::cout << "Iteration " << iteration << ": New Constraints: ";
        for (const auto &constraint : new_constraints)
        {
            std::cout << "(" << constraint.id << ", " << constraint.x << ", " << constraint.y << ", " << constraint.time << ") ";
        }
        std::cout << std::endl;
    }

    return 0;
}
