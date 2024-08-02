#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <utility>
#include <optional>

typedef std::pair<int, int> Pair;

enum Direction
{
    UP,
    DOWN,
    LEFT,
    RIGHT
};

struct Constraint
{
    int id;
    int x;
    int y;
    int time;
    std::optional<int> x2; // Optional second position for edge constraints
    std::optional<int> y2; // Optional second position for edge constraints

    // For edge constraints
    bool IsEdgeConstraint() const { return x2.has_value() && y2.has_value(); }
};

struct State
{
    Pair position;
    Direction direction;
    int time_step;

    bool operator<(const State &other) const
    {
        return std::tie(position, direction, time_step) < std::tie(other.position, other.direction, other.time_step);
    }
};

std::vector<std::vector<int>> AStarAlgorithm(
    const Pair &start,
    const Pair &goal,
    const std::vector<Constraint> &constraints,
    const std::vector<std::vector<int>> &grid);

#endif // ASTAR_H
