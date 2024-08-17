#include "pibt.h"
#include "graph.h"
#include <iostream>
#include <vector>

int main()
{
    //     // // Define starting positions (x, y) and goal positions (x, y)
    std::vector<std::pair<int, int>> starts = {{0, 0}, {1, 1}, {2, 1}, {1, 0}};
    std::vector<std::pair<int, int>> goals = {{2, 2}, {1, 0}, {1, 1}, {1, 2}};

    try
    {
        // Initialize PIBT with the grid, starts, and goals
        pibt planner(3, 3, starts, goals);

        // Run the PIBT algorithm
        planner.run();

        // Print results
        std::cout << "Final positions of agents:\n";
        for (const Agent *agent : planner.agents)
        {
            std::cout << "Agent " << agent->id
                      << " - Start: (" << agent->start->x << ", " << agent->start->y << ")"
                      << " - Goal: (" << agent->goal->x << ", " << agent->goal->y << ")"
                      << " - End: (" << agent->v_next->x << ", " << agent->v_next->y << ")\n"
                      << " - Path: \n";
                      for(auto vertex : agent->Path)
                          std::cout << "(" << vertex.first << ", " << vertex.second << ")";
            std::cout << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}