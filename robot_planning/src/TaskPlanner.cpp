#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <functional>
#include "../include/robotPlanning/TaskPlanner.hpp"

// A* algorithm implementation
AStar::AStar(const Graph& graph, const Point& start, const Point& goal)
    : mGraph(graph), start(start), goal(goal) {}

std::vector<Point> AStar::findPath() {
    std::priority_queue<Node> openSet;
    std::unordered_map<Point, Point> cameFrom;
    std::unordered_map<Point, double> gScores;
    std::unordered_map<Point, double> fScores;

    // Initialize start point
    gScores[start] = 0;
    fScores[start] = heuristic(start, goal);
    openSet.push(Node{ start, 0, fScores[start] });

    while (!openSet.empty()) {
        Node currentNode = openSet.top();
        openSet.pop();
        Point currentPoint = currentNode.point;

        // If we reached the goal, reconstruct the path
        if (currentPoint == goal) {
            return reconstructPath(cameFrom);
        }

        // Iterate over neighbors
        for (const Point& neighbor : mGraph.getEdge(currentPoint)) {
            double tentativeGScore = gScores[currentPoint] + distance(currentPoint, neighbor);
            if (gScores.find(neighbor) == gScores.end() || tentativeGScore < gScores[neighbor]) {
                // We found a better path to this neighbor
                cameFrom[neighbor] = currentPoint;
                gScores[neighbor] = tentativeGScore;
                fScores[neighbor] = gScores[neighbor] + heuristic(neighbor, goal);
                openSet.push(Node{ neighbor, gScores[neighbor], fScores[neighbor] });
            }
        }
    }

    // Return empty vector if no path is found
    return {};
}

// Heuristic function (e.g., Euclidean distance)
double AStar::heuristic(const Point& a, const Point& b) const {
    return sqrt(pow(b.getX() - a.getX(), 2) + pow(b.getY() - a.getY(), 2));
}

// Distance function (e.g., assuming 1-unit distance between adjacent points)
double AStar::distance(const Point& a, const Point& b) const {
    return 1.0;
}

// Reconstruct the path from start to goal using the cameFrom map
std::vector<Point> AStar::reconstructPath(const std::unordered_map<Point, Point>& cameFrom) {
    std::vector<Point> path;
    Point current = goal;
    while (cameFrom.find(current) != cameFrom.end()) {
        path.push_back(current);
        current = cameFrom.at(current);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

// Helper function to print the path
void printPath(const std::vector<Point>& path) {
    if (path.empty()) {
        std::cout << "No path found." << std::endl;
    } else {
        for (const auto& p : path) {
            std::cout << "(" << p.getX() << ", " << p.getY() << ") ";
        }
        std::cout << std::endl;
    }
}
