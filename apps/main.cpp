#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip> 
#include "pibt.h"

int main()
{
    // Define the grid dimensions
    int width = 5;
    int height = 5;

    // Define start and goal positions for the agents
    std::vector<std::vector<int>> starts = {
        {0, 0, 0}, // Agent 0 starts at (0, 0) facing Up
        {4, 0, 1}  // Agent 1 starts at (4, 0) facing Down
    };
    std::vector<std::vector<int>> goals = {
        {4, 4, 0}, // Agent 0's goal is (4, 4), heading Up
        {0, 4, 1}  // Agent 1's goal is (0, 4), heading Down
    };

    // Initialize the PIBT class with the grid dimensions and agent start/goal positions
    PIBT pibt_simulation(width, height, starts, goals);

    // Record the start time
    auto start_time = std::chrono::high_resolution_clock::now();

    // Run the simulation
    pibt_simulation.RunPibt(); // Assuming the method is named `run`

    // Record the end time
    auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the time taken for the simulation
    std::chrono::duration<double> duration = end_time - start_time;

    // Print the status of the agents after running the simulation
    if (pibt_simulation.failed)
    {
        std::cout << "Simulation failed after too many timesteps!" << std::endl;
    }
    else
    {
        std::cout << "Simulation completed successfully!" << std::endl;
    }

    // Optionally, print details about each agent
    pibt_simulation.PrintAgents();

    // Print the time taken to run the PIBT algorithm
    std::cout << "Time taken to run PIBT: "
              << std::fixed << std::setprecision(7)
              << duration.count() << " seconds."
              << std::endl;

    return 0;
}
