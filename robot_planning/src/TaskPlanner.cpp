#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <functional>
#include "graph.h"

// Define the Point structure as a pair of integers (e.g., (x, y)).
typedef std::pair<int, int> Point;

// Define a hash function for Point
namespace std {
    template <>
    struct hash<Point> {
        std::size_t operator()(const Point& p) const {
            return std::hash<int>()(p.first) ^ std::hash<int>()(p.second);
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
    AStar(const Graph& graph, const Point& start, const Point& goal)
        : mGraph(graph), start(start), goal(goal), gScores(), fScores() {}

    std::vector<Point> findPath() {
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

private:
    Graph mGraph;
    Point start;
    Point goal;

    // Heuristic function (e.g., Euclidean distance)
    double heuristic(const Point& a, const Point& b) const {
        return sqrt(pow(b.first - a.first, 2) + pow(b.second - a.second, 2));
    }

    // Distance function (e.g., assuming 1-unit distance between adjacent points)
    double distance(const Point& a, const Point& b) const {
        return 1.0;
    }

    // Reconstruct the path from start to goal using the cameFrom map
    std::vector<Point> reconstructPath(const std::unordered_map<Point, Point>& cameFrom) {
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
};

// Helper function to print the path
void printPath(const std::vector<Point>& path) {
    if (path.empty()) {
        std::cout << "No path found." << std::endl;
    } else {
        for (const auto& p : path) {
            std::cout << "(" << p.first << ", " << p.second << ") ";
        }
        std::cout << std::endl;
    }
}

int main() {
    // Define the graph as an adjacency list (std::map of Point -> vector of Points)
    Graph graph;

    graph.addEdge({0, 0}, {1, 0});
    graph.addEdge({0, 0}, {0, 1});
    graph.addEdge({1, 0}, {0, 0});
    graph.addEdge({1, 0}, {1, 1});
    graph.addEdge({0, 1}, {0, 0});
    graph.addEdge({0, 1}, {1, 1});
    graph.addEdge({1, 1}, {1, 0});
    graph.addEdge({1, 1}, {0, 1});
    graph.addEdge({1, 1}, {2, 2});
    graph.addEdge({2, 2}, {1, 1});

    Point start = {0, 0};
    Point goal = {2, 2};

    AStar astar(graph, start, goal);
    std::vector<Point> path = astar.findPath();

    // Print the path
    printPath(path);

    return 0;
}



/*
// A* Search Algorithm
1.  Initialize the open list
2.  Initialize the closed list
    put the starting node on the open 
    list (you can leave its f at zero)
3.  while the open list is not empty
    a) find the node with the least f on 
       the open list, call it "q"
    b) pop q off the open list
  
    c) generate q's 8 successors and set their 
       parents to q
   
    d) for each successor
        i) if successor is the goal, stop search
        
        ii) else, compute both g and h for successor
          successor.g = q.g + distance between 
                              successor and q
          successor.h = distance from goal to 
          successor (This can be done using many 
          ways, we will discuss three heuristics- 
          Manhattan, Diagonal and Euclidean 
          Heuristics)
          
          successor.f = successor.g + successor.h
        iii) if a node with the same position as 
            successor is in the OPEN list which has a 
           lower f than successor, skip this successor
        iV) if a node with the same position as 
            successor  is in the CLOSED list which has
            a lower f than successor, skip this successor
            otherwise, add  the node to the open list
     end (for loop)
  
    e) push q on the closed list
    end (while loop)
*/