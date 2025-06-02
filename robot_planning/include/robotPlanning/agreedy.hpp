#ifndef DF813D97_1DBB_4C78_8A5A_992462545456
#define DF813D97_1DBB_4C78_8A5A_992462545456

#include "graph.hpp"
#include "point.hpp"
#include "victim.hpp"
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>

// Define a hash function for Point
namespace std {
    template <>
    struct hash<Point> {
        std::size_t operator()(const Point& p) const {
            return std::hash<double>()(p.getX()) ^ (std::hash<double>()(p.getY()) << 1);
        }
    };
}

// Для хеширования пары Point-ов
struct PointPairHash {
    std::size_t operator()(const std::pair<Point, Point>& p) const {
        return std::hash<Point>()(p.first) ^ (std::hash<Point>()(p.second) << 1);
    }
};

struct Node {
    Point point;
    double gCost; // cost from start to current node
    double hCost; // heuristic cost to goal
    double totalValue; // total rescued victim value so far
    std::unordered_set<Point> visitedVictims; // set of visited victim points

    bool operator<(const Node& other) const {
        double thisF = gCost + hCost;
        double otherF = other.gCost + other.hCost;
        if (thisF == otherF)
            return totalValue < other.totalValue;  // Больше ценность — выше приоритет
        return thisF > otherF;  // Меньшее f — выше приоритет
    }

};
class AStarGreedy {
public:
    AStarGreedy(Graph& graph, const std::vector<Victim>& victims,
                const Point& start, const Point& goal, double timeLimit);

    std::vector<Point> findBestPath(double& totalValueCollected);

private:
    Graph& mGraph;
    std::vector<Victim> mVictims;
    Point mStart, mGoal;
    double mTimeLimit;

    double pathCostForOrder(const std::vector<Point>& order);
    void twoOptOptimization(std::vector<Point>& order);

    std::unordered_map<std::pair<Point, Point>, std::vector<Point>, PointPairHash> mPaths;
    std::unordered_map<std::pair<Point, Point>, double, PointPairHash> mCosts;

    std::vector<Point> aStar(const Point& start, const Point& goal, double& pathCost);
    double heuristic(const Point& a, const Point& b);

    void buildMetaGraph();
    const Victim* findVictimAt(const Point& p) const;
};

#endif /* DF813D97_1DBB_4C78_8A5A_992462545456 */
