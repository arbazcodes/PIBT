#pragma once

#include <graph.h>
#include <vector>
#include <unordered_map>

// PIBT agent
struct Agent
{
    int id;
    Vertex *v_now;
    Vertex *v_next;
    Vertex *start;
    Vertex *goal;
    float priority;
    bool reached_goal;
    Direction current_direction;
    std::vector<std::vector<int>> Path;

    Agent(int _id, Vertex *_vnow, Vertex *_vnext, Vertex *_start, Vertex *_goal, float _priority, bool _reached_goal, Direction _current_direction) : id(_id), v_now(_vnow), v_next(_vnext), start(_start), goal(_goal), priority(_priority), reached_goal(_reached_goal), current_direction(_current_direction)
    {
        Path = {{_start->x, _start->y, _start->direction}, {_start->x, _start->y, _start->direction}};
    }
};

// Alias for a collection of agents
using Agents = std::vector<Agent *>;

// PIBT class
class PIBT
{
public:
    PIBT(int w, int h,
         const std::vector<std::vector<int>> &starts,
         const std::vector<std::vector<int>> &goals);
    ~PIBT();

    int HeuristicDistance(const Vertex *start, const Vertex *goal);
    Agent * FindConflictingAgent(const Vertex *v, const Agent *agent);
    bool AllReached();
    void SortAgentsById();
    void RunPibt();
    bool PibtAlgorithm(Agent *ai, Agent *aj = nullptr);
    void PrintAgents();
    
    int timesteps = 0;
    bool failed = false;
    Agents agents;
    Graph graph;
    std::unordered_map<Vertex *, Agent *> occupied_now;
    std::unordered_map<Vertex *, Agent *> occupied_next;
};