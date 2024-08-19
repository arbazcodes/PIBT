#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <iterator>
#include <set>
#include "pibt.h"

// Function to generate unique random positions within grid bounds
std::vector<std::pair<int, int>> GenerateUniqueRandomPositions(int num_positions, int grid_rows, int grid_cols, std::mt19937 &rng)
{
    if (num_positions > grid_rows * grid_cols)
    {
        throw std::invalid_argument("Number of positions requested exceeds the number of available grid cells.");
    }

    std::vector<std::pair<int, int>> all_positions;
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

// Function to ensure unique start and goal positions
void EnsureUniqueStartAndGoal(std::vector<std::pair<int, int>> &starts, std::vector<std::pair<int, int>> &goals, std::mt19937 &rng, int grid_rows, int grid_cols)
{
    std::set<std::pair<int, int>> start_set;
    std::set<std::pair<int, int>> goal_set;

    // Generate unique starts
    while (start_set.size() < starts.size())
    {
        auto new_starts = GenerateUniqueRandomPositions(starts.size(), grid_rows, grid_cols, rng);
        start_set.insert(new_starts.begin(), new_starts.end());
    }
    starts.assign(start_set.begin(), start_set.end());

    // Generate unique goals
    while (goal_set.size() < goals.size())
    {
        auto new_goals = GenerateUniqueRandomPositions(goals.size(), grid_rows, grid_cols, rng);
        goal_set.insert(new_goals.begin(), new_goals.end());
    }
    goals.assign(goal_set.begin(), goal_set.end());

    // Ensure no start position is the same as any goal position
    std::set<std::pair<int, int>> all_positions = start_set;
    all_positions.insert(goal_set.begin(), goal_set.end());

    // Regenerate goals if there are overlaps with start positions
    while (goal_set.size() < goals.size())
    {
        auto new_goals = GenerateUniqueRandomPositions(goals.size(), grid_rows, grid_cols, rng);
        for (const auto &goal : new_goals)
        {
            if (all_positions.find(goal) == all_positions.end())
            {
                goal_set.insert(goal);
                all_positions.insert(goal);
            }
        }
    }
    goals.assign(goal_set.begin(), goal_set.end());
}

int main()
{
    const int num_agents = 36; // Number of agents
    const int width = 6;
    const int height = 6;
    const int num_iterations = 10000; // Number of iterations to run

    // Setup random number generation
    std::random_device rd;
    std::mt19937 rng(rd());

    for (int iteration = 0; iteration < num_iterations; ++iteration)
    {
        // Generate unique random sources and destinations
        auto starts = GenerateUniqueRandomPositions(num_agents, height, width, rng);
        auto goals = GenerateUniqueRandomPositions(num_agents, height, width, rng);

        // Ensure that starts and goals are different
        EnsureUniqueStartAndGoal(starts, goals, rng, height, width);

        std::cout << "Iteration: " << iteration << std::endl;

        // for (int i = 0; i < starts.size(); i++)
        // {
        //     std::cout << "Start: (" << starts[i].first << ", " << starts[i].second << ")---" << "Goal: (" << goals[i].first << ", " << goals[i].second << ")" << std::endl;
        // }


        try
        {

            pibt planner(width, height, starts, goals);

            // Run the PIBT algorithm
            planner.run();

            // // Print results
            // std::cout << "Final positions of agents:\n";
            // for (const Agent *agent : planner.agents)
            // {
            //     std::cout << "Agent " << agent->id
            //             //   << " - Start: (" << agent->start->x << ", " << agent->start->y << ")"
            //             //   << " - Goal: (" << agent->goal->x << ", " << agent->goal->y << ")"
            //             //   << " - End: (" << agent->v_next->x << ", " << agent->v_next->y << ")\n"
            //               << " - Path: \n";
            //     for (auto vertex : agent->Path)
            //         std::cout << "(" << vertex[0] << ", " << vertex[1] <<", " << vertex[2] << ")";
            //     std::cout << std::endl;
            // }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error: " << e.what() << '\n';
            return 1;
        }
    }

    return 0;
}
