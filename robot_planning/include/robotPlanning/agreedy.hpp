#ifndef DF813D97_1DBB_4C78_8A5A_992462545456
#define DF813D97_1DBB_4C78_8A5A_992462545456

#include "graph.hpp"
#include "point.hpp"
#include "victim.hpp"
#include "obstacles.hpp"
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

struct PathOption {
    std::vector<Point> order;
    double totalCost;
    double totalValue;
    double score; // value / cost or similar
};

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
                const Point& start, const Point& goal, double timeLimit, std::vector<Obstacle> obstacles = {}, std::vector<Point> borders = {});

    double angleBetween(const Point& a, const Point& b, const Point& c);
    std::vector<Point> run(double& totalValueCollected, int attempt);
    std::vector<Point> findBestPath(double& totalValueCollected, int attempt);

    //void addEdgePenalty(const Point& p1, const Point& p2, double penalty); 
    void addEdgePenaltyClosest(const Point& rawP1, const Point& rawP2, double penalty) ;
    void addVictimPenaltyFromSegment(const Point& rawP1, const Point& rawP2, std::vector<Point>& visitedVictims) ;
    void addVictimPenalty(const Point& rawP, double penalty) ;

    void buildMetaGraph();

private:
    Graph& mGraph;
    std::vector<Victim> mVictims;
    Point mStart, mGoal;
    double mTimeLimit;
    std::vector<Obstacle> mObstacles;
    std::vector<Point> mBorders;

    std::unordered_map<Point, double> slowVictimPenalties_;
    std::unordered_map<std::pair<Point, Point>, double, PointPairHash> mEdgePenalties; //
    double estimatePathTime(const std::vector<Point>& path) const;

    Point findClosestGraphNode(const Point& query) const;

    double pathCostForOrder(const std::vector<Point>& order);
    void twoOptOptimization(std::vector<Point>& order);
    bool collidesWithObstacleOrBorder(const Point& p1, const Point& p2) const ;
    double distanceToClosestBorder(const Point& p) const ;

    std::unordered_map<std::pair<Point, Point>, std::vector<Point>, PointPairHash> mPaths;
    std::unordered_map<std::pair<Point, Point>, double, PointPairHash> mCosts;

    std::vector<Point> aStar(const Point& start, const Point& goal, double& pathCost);
    double heuristic(const Point& a, const Point& b);

    bool isTooCloseToBorder(const Point& p, double margin) const;

    const Victim* findVictimAt(const Point& p) const;
};

#endif /* DF813D97_1DBB_4C78_8A5A_992462545456 */
