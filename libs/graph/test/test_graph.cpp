#include <gtest/gtest.h>
#include "graph.h"

class GraphTest : public ::testing::Test {
protected:
    // You can set up a graph of fixed size for the tests
    Graph graph = Graph(5, 5);

    virtual void SetUp() override { srand(time(NULL)); }

};

// Test 1: Verify graph dimensions are correctly initialized
TEST_F(GraphTest, InitializationTest) {
    EXPECT_EQ(graph.width, 5);
    EXPECT_EQ(graph.height, 5);
}

// Test 2: Verify neighbor retrieval for a specific vertex
TEST_F(GraphTest, GetNeighborsTest) {
    Vertex *vertex = nullptr;
    // Find a vertex at (2, 2)
    for (Vertex *v : graph.locations) {
        if (v->x == 2 && v->y == 2) {
            vertex = v;
            break;
        }
    }

    // Get neighbors for (2, 2)
    std::vector<Vertex *> neighbors = graph.GetNeighbors(vertex);
    
    // Expect there to be 4 neighbors (if no boundary issues)
    EXPECT_EQ(neighbors.size(), 4);

    // Check if all the directions are correctly set
    EXPECT_EQ(neighbors[0]->direction, Direction::Up);
    EXPECT_EQ(neighbors[1]->direction, Direction::Down);
    EXPECT_EQ(neighbors[2]->direction, Direction::Left);
    EXPECT_EQ(neighbors[3]->direction, Direction::Right);
}

// Test 3: Verify boundary conditions for neighbors
TEST_F(GraphTest, GetNeighborsBoundaryTest) {
    Vertex *vertex = nullptr;
    // Find a vertex at the top-left corner (0, 0)
    for (Vertex *v : graph.locations) {
        if (v->x == 0 && v->y == 0) {
            vertex = v;
            break;
        }
    }

    // Get neighbors for (0, 0)
    std::vector<Vertex *> neighbors = graph.GetNeighbors(vertex);
    
    // Expect only 2 neighbors (down and right) due to boundary limits
    EXPECT_EQ(neighbors.size(), 2);

    // Check if the directions are correct
    EXPECT_EQ(neighbors[0]->direction, Direction::Down);
    EXPECT_EQ(neighbors[1]->direction, Direction::Right);
}

// Test 4: Verify direction to string conversion
TEST_F(GraphTest, DirectionToStringTest) {
    EXPECT_EQ(graph.DirectionToString(Direction::Up), "UP");
    EXPECT_EQ(graph.DirectionToString(Direction::Down), "DOWN");
    EXPECT_EQ(graph.DirectionToString(Direction::Left), "LEFT");
    EXPECT_EQ(graph.DirectionToString(Direction::Right), "RIGHT");
    EXPECT_EQ(graph.DirectionToString(Direction::None), "INVALID");
}

// Test 5: Verify invalid direction handling
TEST_F(GraphTest, InvalidDirectionTest) {
    // Test an invalid direction that is out of the defined range
    Direction invalidDirection = static_cast<Direction>(-1);  // Invalid direction
    EXPECT_EQ(graph.DirectionToString(invalidDirection), "INVALID");
}

// Test 6: Ensure proper cleanup in the destructor
TEST_F(GraphTest, DestructorTest) {
    // Check if the locations set is empty after destruction
    {
        Graph tempGraph(5, 5);
        // Check that the graph was initialized with vertices
        EXPECT_EQ(tempGraph.locations.size(), 25);
    }
}
