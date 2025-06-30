#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <functional>
#include "../include/robotPlanning/AStar.hpp"

// A* algorithm implementation
AStar::AStar(const Graph& graph, const Point& start, const Point& goal)
    : mGraph(graph), start(start), goal(goal) {}

std::vector<Point> AStar::findPath(const std::vector<Victim>& victims) {
    std::priority_queue<Node> openSet;
    std::unordered_map<Point, Point> cameFrom;
    std::unordered_map<Point, double> gScores;
    std::unordered_map<Point, double> fScores;

    // Initialize start point
    gScores[start] = 0;
    fScores[start] = heuristic(start, goal);
    openSet.push(Node{start, 0, fScores[start] });

    while (!openSet.empty()) {
        Node currentNode = openSet.top();
        openSet.pop();
        Point currentPoint = currentNode.point;

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

        //print currentNode
        std::cout << "Exploring: " << currentPoint.toString()
                  << " | g: " << gScores[currentPoint]
                  << " | f: " << fScores[currentPoint] << std::endl;

        // If we reached the goal, reconstruct the path
        if (currentPoint == goal) {
            return reconstructPath(cameFrom);
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
 
/*
#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <unordered_set>
#include <algorithm>

#include "../include/robotPlanning/victim.hpp"
#include "../include/robotPlanning/AStar.hpp"

// Constructor
AStar::AStar(const Graph& graph, const Point& start, const Point& goal)
    : mGraph(graph), start(start), goal(goal) {}

std::vector<Point> AStar::findPath(const std::vector<Victim>& victims, double Tmax) {

    std::priority_queue<Node> openSet;
    std::unordered_map<std::string, double> gScores;
    std::unordered_map<std::string, std::pair<std::string, Point>> cameFrom;

    std::cout << "Start: " << start.toString() << "\n";
    std::cout << "Goal: " << goal.toString() << "\n";
    std::cout << "Victims: " << victims.size() << "\n";

    std::unordered_set<Point> initialVisited;
    gScores[start.toString()] = 0.0;
    double h = heuristic(start, goal);

    openSet.push(Node{start, 0.0, h, 0.0, initialVisited});

    std::string lastKey;

    while (!openSet.empty()) {

        Node currentNode = openSet.top();
        openSet.pop();
        Point currentPoint = currentNode.point;

        std::cout << "Exploring: " << currentPoint.toString()
                  << " | g: " << currentNode.gCost
                  << " | value: " << currentNode.totalValue
                  << " | visitedVictims: " << currentNode.visitedVictims.size() << std::endl;

        // Ключ состояния: позиция + отсортированные жертвы
        std::vector<std::string> keyParts;
        keyParts.push_back(currentPoint.toString());
        for (const auto& vp : currentNode.visitedVictims)
            keyParts.push_back(vp.toString());
        std::sort(keyParts.begin() + 1, keyParts.end());

        std::string currentKey;
        for (const auto& k : keyParts)
            currentKey += "|" + k;

        if (currentPoint == goal) {
            std::cout << "Reached goal: " << goal.toString() << std::endl;
            return reconstructPath(cameFrom, currentKey);
        }

        const std::vector<Point>& neighbors = mGraph.getEdge(currentPoint);

        for (const Point& neighbor : neighbors) {
            double stepCost = distance(currentPoint, neighbor);
            double tentativeGScore = currentNode.gCost + stepCost;

            // 💥 Проверка ограничения по времени
            if (tentativeGScore > Tmax){
                std::cout << "Tentative G-Score exceeds Tmax: " << tentativeGScore << " > " << Tmax << std::endl;
                continue;  // Пропускаем эту соседнюю точку
            } 

            std::unordered_set<Point> newVisitedVictims = currentNode.visitedVictims;
            double newTotalValue = currentNode.totalValue;

            for (const Victim& v : victims) {
                Point victimPos(v.x, v.y);

                if (distance(neighbor, victimPos) <= v.radius && distance(neighbor, victimPos) <= (4.0) && 
                    newVisitedVictims.find(victimPos) == newVisitedVictims.end()) {
                    std::cout << "Visiting victim at: " << victimPos.toString() << " with value: " << v.radius << std::endl;
                    newVisitedVictims.insert(victimPos);
                    newTotalValue += v.radius;
                }
            }

            std::vector<std::string> neighborKeyParts;
            neighborKeyParts.push_back(neighbor.toString());
            for (const auto& vp : newVisitedVictims)
                neighborKeyParts.push_back(vp.toString());
            std::sort(neighborKeyParts.begin() + 1, neighborKeyParts.end());

            std::string neighborKey;
            for (const auto& k : neighborKeyParts)
            {
                neighborKey += "|" + k;
            }

            if (gScores.find(neighborKey) == gScores.end() || tentativeGScore < gScores[neighborKey]) {
                gScores[neighborKey] = tentativeGScore;
                double hScore = heuristic(neighbor, goal);
                Node nextNode{neighbor, tentativeGScore, hScore, newTotalValue, newVisitedVictims};
                openSet.push(nextNode);
                cameFrom[neighborKey] = {currentKey, currentPoint};
            }
        }
        lastKey = currentKey;  // Save the last key for path reconstruction
    }

    std::vector<Point> final = reconstructPath(cameFrom, lastKey);
    std::cout << "Final path reconstructed with " << final.size() << " points." << std::endl;
    for (const auto& p : final) {
        std::cout << p.toString() << " ";
    }
    std::cout << std::endl;
    return {};  // Return empty vector if no path is found
}


// Euclidean heuristic
double AStar::heuristic(const Point& a, const Point& b) const {
    return std::sqrt(std::pow(b.getX() - a.getX(), 2) + std::pow(b.getY() - a.getY(), 2));
}

// Distance between adjacent points (default uniform cost)
double AStar::distance(const Point& a, const Point& b) const {
    // Replace with actual edge cost if available
    return heuristic(a,b);
}

// Reconstruct path from cameFrom map
// std::vector<Point> AStar::reconstructPath(const std::unordered_map<Point, Point>& cameFrom) {
    std::vector<Point> path;
    Point current = goal;
    while (cameFrom.find(current) != cameFrom.end()) {
        path.push_back(current);
        current = cameFrom.at(current);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
//} 

std::vector<Point> AStar::reconstructPath(const std::unordered_map<std::string, std::pair<std::string, Point>>& cameFrom,
    const std::string& currentKey) 
{
    std::cout << "Reconstructing path... cameFrom:" << std::endl;
    for (const auto& entry : cameFrom) {
        std::cout << "Key: " << entry.first << " | Parent Key: " << entry.second.first 
                  << " | Point: " << entry.second.second.toString() << std::endl;
    }
    
    std::cout << "Reconstructing path from key: " << currentKey << std::endl;

    std::vector<Point> path;
    std::string key = currentKey;

    // Trace back using the key history
    while (cameFrom.find(key) != cameFrom.end()) {
        // Extract the point from the cameFrom map
        Point point = cameFrom.at(key).second;
        path.push_back(point);
        key = cameFrom.at(key).first;
    }

    // Add the start point
    path.push_back(start);
    std::cout << "Start point added: " << start.toString() << std::endl;
    std::reverse(path.begin(), path.end());
    std::cout << "Path reconstructed with " << path.size() << " points." << std::endl;
    for (const auto& p : path) {
        std::cout << p.toString() << " ";
    }
    std::cout << std::endl;
    // Return the reconstructed path
    if (path.empty()) {
        std::cout << "No path found." << std::endl;
    }
    return path;
}


// Print path helper
void printPath(const std::vector<Point>& path) {
    if (path.empty()) {
        std::cout << "No path found." << std::endl;
    } else {
        for (const auto& p : path) {
            std::cout << "(" << p.getX() << ", " << p.getY() << ") ";
        }
        std::cout << std::endl;
    }
}*/