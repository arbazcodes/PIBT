#include <iostream>
#include <stdexcept>
#include "graph.h"

Graph::Graph(int w, int h)
    : width(w), height(h)
{
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            Vertex *v = new Vertex(x, y, Direction::Up);
            locations.insert(v);
        }
    }
}

Graph::~Graph()
{
    for (Vertex *v : locations)
    {
        delete v;
    }
    locations.clear();
}

std::vector<Vertex *> Graph::GetNeighbors(const Vertex *v)
{
    std::vector<Vertex *> neighbors;

    int dx[] = {0, 0, -1, 1};
    int dy[] = {-1, 1, 0, 0};

    std::vector<Direction> direction_vector = {Direction::Up, Direction::Down, Direction::Left, Direction::Right};

    for (int i = 0; i < 4; ++i)
    {
        int nx = v->x + dx[i];
        int ny = v->y + dy[i];

        if (nx >= 0 && ny >= 0 && nx < width && ny < height)
        {
            Vertex *neighbor = nullptr;
            for (Vertex *vert : locations)
            {
                if (vert->x == nx && vert->y == ny)
                {
                    neighbor = vert;
                    neighbor->direction = direction_vector[i];
                    break;
                }
            }
            if (neighbor)
            {
                neighbors.push_back(neighbor);
            }
        }
    }

    return neighbors;
}

std::string Graph::DirectionToString(Direction direction) {
    switch (direction)
    {
    case 0:
        return "UP";
        break;
    case 1:
        return "DOWN";
    case 2:
        return "LEFT";
    case 3:
        return "RIGHT";
    default:
        return "INVALID";
    }

    // Shouldn't reach here
    return "INVALID";
}