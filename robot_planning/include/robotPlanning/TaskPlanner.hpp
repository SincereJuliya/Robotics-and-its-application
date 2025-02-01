#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include "graph.hpp"
#include "point.hpp"

/// Define a hash function for Point
namespace std {
    template <>
    struct hash<Point> {
        std::size_t operator()(const Point& p) const {
            return std::hash<double>()(p.getX()) ^ std::hash<double>()(p.getY());
        }
    };
}

// Define a structure to store the A* node information
struct Node {
    Point point;
    double gCost;  // Cost to reach this node from the start
    double hCost;
    double fCost() const { return gCost + hCost; }
    bool operator<(const Node& other) const {
        return fCost() > other.fCost(); // Priority queue needs reverse ordering
    }
};

// A* algorithm implementation
class AStar {
public:
    AStar(const Graph& graph, const Point& start, const Point& goal);

    // + service to dubins
    // + publ to topic 'followPath'

    std::vector<Point> findPath();

private:
    Graph mGraph;
    Point start;
    Point goal;

    // Heuristic function (e.g., Euclidean distance)
    double heuristic(const Point& a, const Point& b) const;

    // Distance function (e.g., assuming 1-unit distance between adjacent points)
    double distance(const Point& a, const Point& b) const;

    // Reconstruct the path from start to goal using the cameFrom map
    std::vector<Point> reconstructPath(const std::unordered_map<Point, Point>& cameFrom);
};

// Helper function to print the path
void printPath(const std::vector<Point>& path);

#endif  // ASTAR_HPP
