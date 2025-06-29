#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <cmath>
#include "graph.hpp"
#include "point.hpp"
#include "IPathPlanner.hpp"

#include "victim.hpp"

// Define a hash function for Point
namespace std {
    template <>
    struct hash<Point> {
        std::size_t operator()(const Point& p) const {
            return std::hash<double>()(p.getX()) ^ (std::hash<double>()(p.getY()) << 1);
        }
    };
}

struct Node {
    Point point;
    double gCost; // cost from start to current node
    double hCost; // heuristic cost to goal
    double totalValue; // total rescued victim value so far
    //std::unordered_set<Point> visitedVictims; // set of visited victim points

    bool operator<(const Node& other) const {
        double thisF = gCost + hCost;
        double otherF = other.gCost + other.hCost;
        if (thisF == otherF)
            return totalValue < other.totalValue;  // Больше ценность — выше приоритет
        return thisF > otherF;  // Меньшее f — выше приоритет

    }

};

// A* algorithm implementation
class AStar : public IPathPlanner {
public:
    AStar() = default;

    AStar(const Graph& graph, const Point& start, const Point& goal);

    // Override: find path considering victims
    std::vector<Point> findPath(const std::vector<Victim>& victims) override;

private:
    Graph mGraph;
    Point start;
    Point goal;

    double heuristic(const Point& a, const Point& b) const;

    double distance(const Point& a, const Point& b) const;


    double AStarGreedy::estimatePathTime(const std::vector<Point>& path) const;

    //std::vector<Point> reconstructPath(const std::unordered_map<std::string, std::pair<std::string, Point>>& cameFrom,
    //const std::string& currentKey
        /* const std::unordered_map<Point, Point>& cameFrom );*/

    std::vector<Point> reconstructPath(
        const std::unordered_map<Point, Point>& cameFrom );
};

// Helper function to print the path
void printPath(const std::vector<Point>& path);

#endif  // ASTAR_HPP
