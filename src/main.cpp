#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <iterator>
#include <set>
#include <chrono> // Include chrono for timing
#include "pibt.h"

// Function to generate unique random positions within grid bounds
std::vector<std::vector<int>> GenerateUniqueRandomPositions(int num_positions, int grid_rows, int grid_cols, std::mt19937 &rng)
{
    if (num_positions > grid_rows * grid_cols)
    {
        throw std::invalid_argument("Number of positions requested exceeds the number of available grid cells.");
    }

    std::vector<std::vector<int>> all_positions;
    for (int row = 0; row < grid_rows; ++row)
    {
        for (int col = 0; col < grid_cols; ++col)
        {
            all_positions.push_back({row, col, 3});
        }
    }

    std::shuffle(all_positions.begin(), all_positions.end(), rng);
    all_positions.resize(num_positions);
    return all_positions;
}

// Function to ensure unique start and goal positions
void EnsureUniqueStartAndGoal(std::vector<std::vector<int>> &starts, std::vector<std::vector<int>> &goals, std::mt19937 &rng, int grid_rows, int grid_cols)
{
    std::set<std::vector<int>> start_set;
    std::set<std::vector<int>> goal_set;

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
    std::set<std::vector<int>> all_positions = start_set;
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
    const int num_agents = 20; // Number of agents
    const int width = 6;
    const int height = 6;
    const int num_iterations = 1000; // Number of iterations to run

    // Setup random number generation
    std::random_device rd;
    std::mt19937 rng(rd());

    int failure_count = 0;
    std::chrono::duration<double> total_duration(0);

    for (int iteration = 0; iteration < num_iterations; ++iteration)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        // Generate unique random sources and destinations
        auto starts = GenerateUniqueRandomPositions(num_agents, height, width, rng);
        auto goals = GenerateUniqueRandomPositions(num_agents, height, width, rng);

        // Ensure that starts and goals are different
        // EnsureUniqueStartAndGoal(starts, goals, rng, height, width);

        // for (int i = 0; i < starts.size(); i++)
        // {
        //     std::cout << "Start: (" << starts[i][0] << ", " << starts[i][1] << ", " << starts[i][2] << ")---" << "Goal: (" << goals[i][0] << ", " << goals[i][1] << ", " << goals[i][2] << ")" << std::endl;
        // }

        std::cout << "Iteration: " << iteration << std::endl;

        pibt *planner = nullptr;

        try
        {
            int recursive_run = 0;

            while (recursive_run < 10)
            {
                // Create a new planner instance
                planner = new pibt(width, height, starts, goals);
                // Run the PIBT algorithm with a timeout
                planner->timesteps = 0;
                planner->failed = false; // Reset failure flag
                planner->run();

                if (planner->failed)
                {
                    recursive_run += 1;
                    if (recursive_run == 10)
                    {
                        failure_count++;
                        std::cout << "Failed or Timed Out!\n";
                    }
                }
                else
                {
                    break;
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error: " << e.what() << '\n';
            failure_count++;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> iteration_duration = end_time - start_time;
        total_duration += iteration_duration;
        
        // for (auto agent : planner->agents)
        // {
        //     std::cout << "Agent " << agent->id << ": ";
        //     for (const auto &step : agent->Path)
        //     {
        //         std::cout << "(" << step[0] << ", " << step[1] << ", " << step[2] << ")";
        //     }
        //     std::cout << std::endl;
        // }

        // Cleanup
        delete planner;


        std::cout << "Iteration Time: " << iteration_duration.count() << " seconds" << std::endl;
    }

    // Calculate and print the average time per iteration
    double average_time = total_duration.count() / num_iterations;
    std::cout << "Average Time per Iteration: " << average_time << " seconds" << std::endl;
    std::cout << "Passed: " << (num_iterations - failure_count) << " Failed: " << failure_count << " Percentage: " << (float)(((failure_count) / (float)(num_iterations - failure_count)) * 100) << "%" << std::endl;

    return 0;
}

// std::vector<std::pair<int, int>> starts = {
//     {4, 2},
//     {2, 1},
//     {0, 5},
//     {5, 2},
//     {1, 0},
//     {2, 4},
//     {4, 3},
//     {5, 4},
//     {2, 2},
//     {3, 0},
//     {2, 5},
//     {0, 0},
//     {0, 1},
//     {4, 0},
//     {1, 4},
//     {3, 5},
//     {1, 1},
//     {4, 5},
//     {5, 1},
//     {0, 4},
//     {1, 3},
//     {0, 3},
//     {1, 5},
//     {3, 3},
//     {1, 2}};

// std::vector<std::pair<int, int>> goals = {
//     {0, 3},
//     {5, 5},
//     {1, 1},
//     {5, 1},
//     {5, 4},
//     {4, 2},
//     {0, 4},
//     {1, 0},
//     {3, 3},
//     {5, 2},
//     {0, 2},
//     {1, 3},
//     {3, 2},
//     {1, 2},
//     {0, 5},
//     {4, 1},
//     {3, 4},
//     {5, 3},
//     {4, 4},
//     {3, 5},
//     {2, 0},
//     {1, 4},
//     {3, 0},
//     {1, 5},
//     {4, 5}};