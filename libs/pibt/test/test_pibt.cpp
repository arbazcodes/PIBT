#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip> 
#include <vector>
#include "pibt.h"
#include "graph.h"

// A utility function to set up a simple 3x3 grid for testing
std::vector<std::vector<int>> createStartGoalData() {
    // Simple start-goal setup for 3 agents on a 3x3 grid
    return {
        {0, 0, 0}, 
        {0, 1, 1},  
        {0, 2, 2},  
    };
}

// Test case 1: Verify Initialization of Agents
TEST(PIBTTest, InitializeAgents) {
    std::vector<std::vector<int>> starts = {{0, 0, 0}, {0, 1, 1}, {0, 2, 2}};
    std::vector<std::vector<int>> goals = {{2, 2, 0}, {2, 2, 1}, {2, 2, 2}};
    PIBT pibt(3, 3, starts, goals);

    // Check the number of agents
    ASSERT_EQ(pibt.agents.size(), 3);

    // Verify the first agent's start and goal
    Agent* agent1 = pibt.agents[0];
    ASSERT_EQ(agent1->start->x, 0);
    ASSERT_EQ(agent1->start->y, 0);
    ASSERT_EQ(agent1->goal->x, 2);
    ASSERT_EQ(agent1->goal->y, 2);
    ASSERT_EQ(agent1->current_direction, Direction::Up);  // Direction 0

    // Verify the second agent's start and goal
    Agent* agent2 = pibt.agents[1];
    ASSERT_EQ(agent2->start->x, 0);
    ASSERT_EQ(agent2->start->y, 1);
    ASSERT_EQ(agent2->goal->x, 2);
    ASSERT_EQ(agent2->goal->y, 2);
    ASSERT_EQ(agent2->current_direction, Direction::Down);  // Direction 1
}

// Test case 2: Verify Heuristic Distance Calculation
TEST(PIBTTest, HeuristicDistance) {
    std::vector<std::vector<int>> starts = {{0, 0, 0}};
    std::vector<std::vector<int>> goals = {{2, 2, 0}};
    PIBT pibt(3, 3, starts, goals);

    // Check the heuristic distance from (0, 0) to (2, 2)
    Vertex* start = new Vertex(0, 0, (Direction) 0);
    Vertex* goal = new Vertex(2, 2, (Direction) 0);

    int heuristic = pibt.HeuristicDistance(start, goal);
    ASSERT_EQ(heuristic, 4);  // Manhattan distance between (0, 0) and (2, 2) is 4
}

// Test case 3: Verify Conflict Detection
TEST(PIBTTest, ConflictDetection) {
    std::vector<std::vector<int>> starts = {{0, 0, 0}, {0, 0, 0}};
    std::vector<std::vector<int>> goals = {{1, 1, 0}, {1, 2, 0}};
    PIBT pibt(3, 3, starts, goals);

    Agent* agent1 = pibt.agents[0];
    Agent* agent2 = pibt.agents[1];

    // Check if agent1 and agent2 are in conflict at the same position
    Vertex* v1 = agent1->v_now;
    Vertex* v2 = agent2->v_now;
    Agent* conflicting_agent = pibt.FindConflictingAgent(v1, agent1);
    ASSERT_EQ(conflicting_agent, agent2);
}

// Test case 4: Verify Pathfinding Behavior
TEST(PIBTTest, PathfindingBehavior) {
    std::vector<std::vector<int>> starts = {{0, 0, 0}};
    std::vector<std::vector<int>> goals = {{2, 2, 0}};
    PIBT pibt(3, 3, starts, goals);

    Agent* agent1 = pibt.agents[0];

    // Simulate one step of the algorithm
    pibt.RunPibt();

    // After one step, agent should have moved to a neighboring vertex
    ASSERT_NE(agent1->v_now, agent1->start);  // The agent should not be at the starting point anymore
    ASSERT_TRUE(agent1->v_next != nullptr);   // There should be a valid next position for the agent
}

// Test case 5: Verify Goal Reaching Condition
TEST(PIBTTest, GoalReachingCondition) {
    std::vector<std::vector<int>> starts = {{0, 0, 0}};
    std::vector<std::vector<int>> goals = {{0, 2, 0}};
    PIBT pibt(3, 3, starts, goals);

    // Run the algorithm until the agent reaches its goal
    while (!pibt.AllReached()) {
        pibt.RunPibt();
    }

    // Verify that the agent reached its goal
    Agent* agent1 = pibt.agents[0];
    ASSERT_TRUE(agent1->reached_goal);
    ASSERT_EQ(agent1->v_now->x, 0);
    ASSERT_EQ(agent1->v_now->y, 2);
}

TEST(PIBTTest, RunFor100Iterations) {
    // Optionally, set up some default values for the grid and agents
    int width = 5;
    int height = 5;
    std::vector<std::vector<int>> starts = {
        {0, 0, 0}, // Agent 0 starts at (0, 0) facing Up
        {4, 0, 1}  // Agent 1 starts at (4, 0) facing Down
    };
    std::vector<std::vector<int>> goals = {
        {4, 4, 0}, // Agent 0's goal is (4, 4), heading Up
        {0, 4, 1}  // Agent 1's goal is (0, 4), heading Down
    };

    PIBT* pibt_simulation;
    // Record the start time
    auto start_time = std::chrono::high_resolution_clock::now();

    // Run the simulation for 100 iterations
    int max_iterations = 100;
    bool reached_goal = false;

    for (int i = 0; i < max_iterations; ++i) {
        // Initialize PIBT simulation
        pibt_simulation = new PIBT(width, height, starts, goals);
        pibt_simulation->RunPibt();  // Running the PIBT simulation for each iteration

        ASSERT_FALSE(pibt_simulation->failed);
        // Check if all agents have reached their goal
        for (auto agent : pibt_simulation->agents) {
            ASSERT_TRUE(agent->reached_goal);
        } 


        delete pibt_simulation;
    }

    // Record the end time
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    

    // Print the time taken to run the PIBT algorithm
    std::cout << "Time taken to run PIBT for " << max_iterations << " iterations: "
              << std::fixed << std::setprecision(7)
              << duration.count() << " seconds."
              << std::endl;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
