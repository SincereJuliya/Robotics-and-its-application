/* #ifndef ASTAR_HPP
#define ASTAR_HPP

#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include "graph.hpp"
#include "point.hpp"
#include "IPathPlanner.hpp"

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

    double totalValue; // Value of rescued victims
    std::unordered_set<Point> visitedVictims;

    bool operator<(const Node& other) const {
        //return fCost() > other.fCost(); // Priority queue needs reverse ordering
        // Higher value comes first, then lower cost
        if (totalValue == other.totalValue)
            return fScore > other.fScore; // Min-heap on fScore
        return totalValue < other.totalValue; // Max-heap on value
    }
};

// A* algorithm implementation
class AStar : public IPathPlanner {
public:
    AStar() = default;
    std::vector<Point> findPath(const std::vector<Victim>& victims) override;

    AStar(const Graph& graph, const Point& start, const Point& goal);

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
 */

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

    //std::vector<Point> reconstructPath(const std::unordered_map<std::string, std::pair<std::string, Point>>& cameFrom,
    //const std::string& currentKey
        /* const std::unordered_map<Point, Point>& cameFrom );*/

    std::vector<Point> reconstructPath(
        const std::unordered_map<Point, Point>& cameFrom );
};

// Helper function to print the path
void printPath(const std::vector<Point>& path);

#endif  // ASTAR_HPP
