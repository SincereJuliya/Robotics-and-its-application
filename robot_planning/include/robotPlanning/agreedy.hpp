#include "graph.hpp"
#include "point.hpp"
#include "victim.hpp"
#include "obstacles.hpp"
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
#include <limits>
#include <cmath>
#include <set>

/**
 * @brief Hash specialization for Point so it can be used in unordered_map/set.
 */
namespace std {
    template <>
    struct hash<Point> {
        std::size_t operator()(const Point& p) const {
            return std::hash<double>()(p.getX()) ^ (std::hash<double>()(p.getY()) << 1);
        }
    };
}

/**
 * @brief Represents a path option including its value, cost, and order.
 */
struct PathOption {
    std::vector<Point> order;
    double totalCost;
    double totalValue;
    double score; ///< Typically value / cost
};

/**
 * @brief Hash function for std::pair of Points (used for edge representation).
 */
struct PointPairHash {
    std::size_t operator()(const std::pair<Point, Point>& p) const {
        return std::hash<Point>()(p.first) ^ (std::hash<Point>()(p.second) << 1);
    }
};

/**
 * @brief Node structure used in A* search with heuristic, path cost and victim tracking.
 */
struct Node {
    Point point;
    double gCost; ///< Cost from start to this node
    double hCost; ///< Heuristic estimate to goal
    double totalValue; ///< Total value of rescued victims
    std::unordered_set<Point> visitedVictims;

    bool operator<(const Node& other) const {
        double thisF = gCost + hCost;
        double otherF = other.gCost + other.hCost;
        if (thisF == otherF)
            return totalValue < other.totalValue;
        return thisF > otherF;
    }
};

/**
 * @class AStarGreedy
 * @brief A greedy A* planner for rescue missions, considering obstacles, victims, penalties, and time constraints.
 */
class AStarGreedy {
public:
    /**
     * @brief Construct with full parameters.
     */
    AStarGreedy(Graph& graph, const std::vector<Victim>& victims,
                const Point& start, const Point& goal, double timeLimit,
                std::vector<Obstacle> obstacles = {}, std::vector<Point> borders = {});

    /**
     * @brief Default constructor with dummy start/goal.
     */
    AStarGreedy(Graph& graph);

    /**
     * @brief Update planner data.
     */
    void setData(const std::vector<Victim>& victims,
                 const Point& start,
                 const Point& goal,
                 double timeLimit,
                 const std::vector<Obstacle>& obstacles,
                 const std::vector<Point>& borders);

    std::vector<Point> run(double& totalValueCollected, int attempt);
    std::vector<Point> findBestPath(double& totalValueCollected, int attempt);

    void addEdgePenaltyClosest(const Point& rawP1, const Point& rawP2, double penalty);
    void addVictimPenaltyFromSegment(const Point& rawP1, const Point& rawP2, std::vector<Victim>& visitedVictims);
    void addVictimPenalty(const Point& rawP, int attempt, double victimValue = 0.0);

    void buildMetaGraph();

private:
    Graph& mGraph;
    std::vector<Victim> mVictims;
    Point mStart, mGoal;
    double mTimeLimit;
    std::vector<Obstacle> mObstacles;
    std::vector<Point> mBorders;

    std::unordered_map<Point, double> slowVictimPenalties_;
    std::unordered_map<std::pair<Point, Point>, double, PointPairHash> mEdgePenalties;
    std::unordered_map<std::pair<Point, Point>, std::vector<Point>, PointPairHash> mPaths;
    std::unordered_map<std::pair<Point, Point>, double, PointPairHash> mCosts;

    std::vector<Point> aStar(const Point& start, const Point& goal, double& pathCost);
    double heuristic(const Point& a, const Point& b);
    double estimatePathTime(const std::vector<Point>& path) const;
    Point findClosestGraphNode(const Point& query) const;
    double pathCostForOrder(const std::vector<Point>& order);
    void twoOptOptimization(std::vector<Point>& order);
    bool collidesWithObstacleOrBorder(const Point& p1, const Point& p2) const;
    double distanceToClosestBorder(const Point& p) const;
    bool isTooCloseToBorder(const Point& p, double margin) const;
    double angleBetween(const Point& a, const Point& b, const Point& c);
    const Victim* findVictimAt(const Point& p) const;
};

