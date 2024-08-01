#include <iostream>
#include "cbs.h"

int main() {
    std::vector<Pair> starts = {{0, 0}, {0, 2}, {0, 1}, {1, 1}, {1, 0}, {2, 1}, /*{2, 0},*/ {2, 2}
    };
    std::vector<Pair> goals = {{0, 2}, {0, 0}, {1, 1}, {1, 0}, {2, 1}, {1, 2}, /*{0, 1},*/ {2, 0}
    };

    // Define constraints (empty in this case)
    std::vector<Constraint> constraints = {};

    // Define the grid where agents operate
    std::vector<std::vector<int>> grid = {
        {1, 1, 1},
        {1, 1, 1},
        {1, 1, 1}};

    Cbs cbsAlgorithm(grid);

    vector<CostPath> solution = cbsAlgorithm.HighLevel(starts, goals);

    std::cout << "Final Solution Paths and Costs:" << std::endl;
    for (size_t i = 0; i < starts.size(); i++) {
        std::cout << "Agent " << i << " - Cost: " << solution[i].size() << " Path: ";
        for (const auto& step : solution[i]) {
            std::cout << "(" << step[0] << ", " << step[1] << ", " << step[2] << ") ";
        }
        std::cout << std::endl;
    }

    int totalCost = cbsAlgorithm.FindTotalCost(solution);
    std::cout << "Total Cost: " << totalCost << std::endl;

    vector<vector<int>> conflicts = cbsAlgorithm.FindConflicts(solution);
    std::cout << "Conflicts: ";
    for (const auto& conflict : conflicts) {
        std::cout << "(" << conflict[0] << ", " << conflict[1] << ", " << conflict[2] << ", " << conflict[3] << ", " << conflict[4] << ") ";
    }
    std::cout << std::endl;

    vector<Constraint> newConstraints = cbsAlgorithm.GenerateConstraints(conflicts);
    std::cout << "New Constraints: ";
    for (const auto& constraint : newConstraints) {
        std::cout << "(" << constraint.id << ", " << constraint.x << ", " << constraint.y << ", " << constraint.time << ") ";
    }
    std::cout << std::endl;

    return 0;
}
