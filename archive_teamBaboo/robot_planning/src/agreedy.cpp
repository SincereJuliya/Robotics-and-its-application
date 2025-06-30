#include "../include/robotPlanning/agreedy.hpp"

// to see quick
#define VELOCITY 0.26
const float minDistObs = 0.2f;    // Minimum distance to obstacles
const float minDistBorder = 0.2f; // Minimum distance to borders


/**
 * @brief Constructs an AStarGreedy object with default parameters.
 * 
 * This constructor initializes the AStarGreedy object with the provided graph
 * and sets default values for the start and goal positions, time limit, obstacles,
 * and borders. It also outputs a message indicating that the default constructor
 * has been called.
 * 
 * @param graph Reference to the Graph object used for pathfinding.
 */
AStarGreedy::AStarGreedy(Graph &graph)
    : mGraph(graph), mStart(0, 0), mGoal(0, 0), mTimeLimit(60.0), mObstacles(), mBorders()
{
    std::cout << "Default constructor called for AStarGreedy.\n";
}

/**
 * @brief Constructor for the AStarGreedy class.
 *
 * This constructor initializes the AStarGreedy object with the given graph, victims, 
 * start and goal points, time limit, obstacles, and borders. It also builds the meta 
 * graph and calculates paths and costs required for the algorithm.
 *
 * @param graph Reference to the graph representing the environment.
 * @param victims A vector of Victim objects representing the victims to be considered.
 * @param start The starting point of the path.
 * @param goal The goal point of the path.
 * @param timeLimit The time limit for the algorithm to find a solution.
 * @param obstacles A vector of Obstacle objects representing obstacles in the environment.
 * @param borders A vector of Point objects representing the borders of the environment.
 *
 * @note The constructor outputs debug information to the console, including the start 
 *       and goal points, the number of paths and costs calculated, and any errors 
 *       encountered during the meta graph construction.
 */
AStarGreedy::AStarGreedy(Graph &graph, const std::vector<Victim> &victims,
                         const Point &start, const Point &goal, double timeLimit, std::vector<Obstacle> obstacles, std::vector<Point> borders)
    : mGraph(graph), mVictims(victims), mStart(start), mGoal(goal), mTimeLimit(timeLimit),
      mObstacles(obstacles), mBorders(borders)
{
    buildMetaGraph();
    std::cout << "CONSTRUCTOR ASTARGREEDY\n";
    std::cout << "Start: " << mStart.toString() << ", Goal: " << mGoal.toString() << "\n";
    std::cout << "Meta graph built with " << mPaths.size() << " paths.\n";
    std::cout << "Total costs calculated: " << mCosts.size() << "\n";
    if (mPaths.empty() || mCosts.empty())
    {
        std::cerr << "Error: No paths or costs calculated in the meta graph.\n";
    }
    std::cout << "AStarGreedy initialized with time limit: " << mTimeLimit << "\n";
    std::cout << "CONSTRUCTOR ASTARGREEDY CREATED\n";
}

/**
 * @brief Computes the heuristic distance between two points using the Euclidean distance formula.
 * 
 * This function calculates the straight-line distance between two points in a 2D space.
 * It is used to estimate the cost from the current point to the goal.
 * 
 * @param a The first point.
 * @param b The second point.
 * @return The Euclidean distance between point a and point b.
 */
double AStarGreedy::heuristic(const Point &a, const Point &b)
{
    double dx = a.getX() - b.getX();
    double dy = a.getY() - b.getY();
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief Finds the closest graph node to a given query point.
 *
 * This function iterates through all the vertices in the graph and computes
 * the Euclidean distance between the query point and each vertex. It returns
 * the vertex that is closest to the query point.
 *
 * @param query The point for which the closest graph node is to be found.
 * @return The closest graph node to the query point.
 */
Point AStarGreedy::findClosestGraphNode(const Point &query) const
{
    double minDist = std::numeric_limits<double>::infinity();
    Point closest;

    for (const auto &entry : mGraph.getVertices())
    { // getVertices() — множество всех точек графа
        double dist = query.computeEuclideanDistance(entry);
        if (dist < minDist)
        {
            minDist = dist;
            closest = entry;
        }
    }

    return closest;
}

/**
 * @brief Adds a penalty for victims that are in proximity to a given line segment.
 *
 * This function calculates the distance of each victim from a specified line segment
 * and adds victims to the visitedVictims list if they are within a defined proximity threshold.
 *
 * @param rawP1 The starting point of the line segment.
 * @param rawP2 The ending point of the line segment.
 * @param visitedVictims A reference to a vector of victims that have already been visited. 
 *                       Victims within the proximity threshold of the line segment will be added to this vector.
 *
 * @note The proximity threshold is defined as a constant within the function.
 *       Victims are only added to the visitedVictims list if they are not already present in it.
 */
void AStarGreedy::addVictimPenaltyFromSegment(const Point &rawP1, const Point &rawP2, std::vector<Victim> &visitedVictims)
{
    // Distance threshold: how close a victim should be to be affected
    constexpr double victimProximityThreshold = 1.0;

    auto pointToSegmentDistance = [](const Point &p, const Point &a, const Point &b)
    {
        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();

        if (dx == 0 && dy == 0)
            return std::hypot(p.getX() - a.getX(), p.getY() - a.getY());

        double t = ((p.getX() - a.getX()) * dx + (p.getY() - a.getY()) * dy) / (dx * dx + dy * dy);
        t = std::max(0.0, std::min(1.0, t));

        double projX = a.getX() + t * dx;
        double projY = a.getY() + t * dy;

        return std::hypot(p.getX() - projX, p.getY() - projY);
    };

    for (const Victim &v : mVictims)
    {
        Point vp(v.x, v.y);

        double dist = pointToSegmentDistance(vp, rawP1, rawP2);

        if (dist < victimProximityThreshold)
        {
            if (std::find(visitedVictims.begin(), visitedVictims.end(), v) == visitedVictims.end())
            {
                visitedVictims.push_back(v);
            }
        }
    }
}

/**
 * @brief Adds a penalty for a victim point based on the detour cost, attempt count, and victim value.
 *
 * This function calculates a penalty for a victim point by considering the detour cost from the start
 * to the victim point and then to the goal, the number of attempts to handle the victim, and the victim's
 * value. The penalty is adjusted by various factors such as an attempt multiplier and a radius factor,
 * and is capped within a specified range. The calculated penalty is added to the total penalty for the
 * victim point.
 *
 * @param victimPoint The point representing the victim's location.
 * @param attempt The number of attempts made to handle the victim.
 * @param victimValue A value representing the victim's importance or priority.
 *
 * @note If the cost from the start to the victim point or from the victim point to the goal is not available,
 *       the function skips adding the penalty and logs a message.
 * @note The penalty is constrained to a minimum value of 1.0 and a maximum value of 200.0.
 * @note The function logs the details of the penalty calculation and the updated total penalty.
 */
void AStarGreedy::addVictimPenalty(const Point &victimPoint, int attempt, double victimValue)
{
    if (!mCosts.count({mStart, victimPoint}) || !mCosts.count({victimPoint, mGoal}))
    {
        std::cerr << "Skipping penalty: no mCosts for " << victimPoint.toString() << std::endl;
        return;
    }

    double detourCost = mCosts[{mStart, victimPoint}] + mCosts[{victimPoint, mGoal}];
    double basePenalty = detourCost * VELOCITY;

    int cappedAttempt = std::min(attempt, 20);
    double attemptMultiplier = (1.0 + 0.05 * cappedAttempt) * std::pow(1.02, cappedAttempt);

    double radiusFactor = 1.0 - std::min(0.15, victimValue * 0.0003);
    double penalty = basePenalty * attemptMultiplier * radiusFactor * VELOCITY;

    penalty = std::min(penalty, 200.0);
    penalty = std::max(penalty, 1.0);

    const double minimalPenaltyThreshold = 1.0;
    penalty = std::max(penalty, minimalPenaltyThreshold);

    slowVictimPenalties_[victimPoint] += penalty;

    std::cout << "Penalty added for " << victimPoint.toString()
              << " | detourCost: " << detourCost
              << " | attempt: " << attempt
              << " | victimValue: " << victimValue
              << " | penalty: " << penalty
              << " | total penalty: " << slowVictimPenalties_[victimPoint]
              << std::endl;
}

/**
 * @brief Adds a penalty to the edge connecting the closest graph nodes to the given points.
 *
 * This function finds the closest nodes in the graph to the provided points and applies
 * a penalty to the edge connecting these nodes. If the graph is undirected, the penalty
 * is applied in both directions.
 *
 * @param rawP1 The first point in the raw coordinate space.
 * @param rawP2 The second point in the raw coordinate space.
 * @param penalty The penalty value to be added to the edge.
 */
void AStarGreedy::addEdgePenaltyClosest(const Point &rawP1, const Point &rawP2, double penalty)
{
    Point p1 = findClosestGraphNode(rawP1);
    Point p2 = findClosestGraphNode(rawP2);

    mEdgePenalties[{p1, p2}] += penalty;
    mEdgePenalties[{p2, p1}] += penalty; 
}

/**
 * @brief Implements the A* algorithm with greedy heuristics to find the shortest path 
 *        between a start and goal point in a graph.
 * 
 * @param start The starting point of the path.
 * @param goal The goal point of the path.
 * @param pathCost Reference to a double where the cost of the computed path will be stored.
 *                 If no path is found, it will be set to infinity.
 * 
 * @return A vector of Points representing the shortest path from start to goal.
 *         If no path is found, an empty vector is returned.
 * 
 * @details This implementation uses a priority queue to explore nodes with the lowest 
 *          estimated cost first. The cost is calculated as the sum of the actual cost 
 *          from the start to the current node (gScore) and the heuristic estimate to 
 *          the goal. Additional penalties can be applied to edges and vertices to 
 *          influence the pathfinding behavior.
 * 
 * @note The heuristic function used must be admissible (never overestimate the cost) 
 *       to guarantee optimality of the path.
 * 
 * @warning The function assumes that the graph and heuristic are properly defined and 
 *          that the start and goal points exist in the graph.
 */
std::vector<Point> AStarGreedy::aStar(const Point &start, const Point &goal, double &pathCost)
{
    std::unordered_map<Point, double> gScore;
    std::unordered_map<Point, Point> cameFrom;
    std::priority_queue<Node> openSet;

    gScore[start] = 0.0;
    Node startNode{start, 0.0, heuristic(start, goal), 0.0, {}};
    openSet.push(startNode);

    while (!openSet.empty())
    {
        Node current = openSet.top();
        openSet.pop();

        if (current.point == goal)
        {
            std::vector<Point> path;
            Point p = goal;
            while (p != start)
            {
                path.push_back(p);
                p = cameFrom[p];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            pathCost = current.gCost;
            return path;
        }

        for (const Point &neighbor : mGraph.getEdge(current.point))
        {
            // Базовая стоимость перехода
            double baseCost = (heuristic(current.point, neighbor));

            // Добавляем штраф, если есть для ребра (current.point, neighbor)
            double penaltyEdge = 0.0;
            auto it = mEdgePenalties.find({current.point, neighbor});
            if (it != mEdgePenalties.end())
            {
                penaltyEdge = it->second;
            }

            // Штрафы на жертву (на вершину neighbor)
            double penaltyVictim = 0.0;
            auto itVictim = slowVictimPenalties_.find(neighbor);
            if (itVictim != slowVictimPenalties_.end())
            {
                penaltyVictim = itVictim->second;
            }

            double tentativeG = gScore[current.point] + baseCost + penaltyEdge + penaltyVictim;

            if (!gScore.count(neighbor) || tentativeG < gScore[neighbor])
            {
                gScore[neighbor] = tentativeG;
                cameFrom[neighbor] = current.point;
                Node neighborNode{neighbor, tentativeG, heuristic(neighbor, goal), 0.0, {}};
                openSet.push(neighborNode);
            }
        }
    }

    pathCost = std::numeric_limits<double>::infinity();
    return {};
}

/**
 * @brief Checks if a given point is too close to the border of a defined area.
 *
 * This function determines whether a point lies within a specified margin
 * from the borders of a polygonal area. The borders are defined as a series
 * of connected line segments stored in the `mBorders` member variable.
 *
 * @param p The point to check, represented as a `Point` object.
 * @param margin The minimum allowable distance from the border. If the point
 *               is closer than this margin to any border segment, the function
 *               returns true.
 * @return `true` if the point is too close to the border or if the borders
 *         are not properly initialized (less than 2 points). Otherwise, returns `false`.
 *
 * @note If the borders are not initialized (i.e., `mBorders` contains fewer
 *       than 2 points), the function conservatively returns `true`.
 */
bool AStarGreedy::isTooCloseToBorder(const Point &p, double margin) const
{
    if (mBorders.size() < 2)
        return true; // Not initialized, be conservative

    auto pointToSegmentDistance = [](const Point &p, const Point &a, const Point &b)
    {
        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();

        if (dx == 0 && dy == 0)
            return std::hypot(p.getX() - a.getX(), p.getY() - a.getY());

        double t = ((p.getX() - a.getX()) * dx + (p.getY() - a.getY()) * dy) / (dx * dx + dy * dy);
        t = std::max(0.0, std::min(1.0, t));

        double projX = a.getX() + t * dx;
        double projY = a.getY() + t * dy;

        return std::hypot(p.getX() - projX, p.getY() - projY);
    };

    for (size_t i = 0; i < mBorders.size(); ++i)
    {
        const Point &a = mBorders[i];
        const Point &b = mBorders[(i + 1) % mBorders.size()];
        double dist = pointToSegmentDistance(p, a, b);
        if (dist < margin)
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief Calculates the shortest distance from a given point to the closest border segment.
 *
 * This function computes the minimum distance from the specified point to the 
 * nearest line segment defined by the borders of a polygon. The borders are 
 * assumed to be stored in a vector of points, where consecutive points form 
 * the edges of the polygon, and the last point connects back to the first.
 *
 * @param p The point for which the distance to the closest border is calculated.
 * @return The shortest distance from the point to the closest border segment.
 *         Returns 0.0 if the borders are not initialized or malformed (less than 2 points).
 */
double AStarGreedy::distanceToClosestBorder(const Point &p) const
{
    if (mBorders.size() < 2)
        return 0.0; // Not initialized or malformed

    auto pointToSegmentDistance = [](const Point &p, const Point &a, const Point &b)
    {
        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();

        if (dx == 0 && dy == 0)
            return std::hypot(p.getX() - a.getX(), p.getY() - a.getY());

        double t = ((p.getX() - a.getX()) * dx + (p.getY() - a.getY()) * dy) / (dx * dx + dy * dy);
        t = std::max(0.0, std::min(1.0, t));

        double projX = a.getX() + t * dx;
        double projY = a.getY() + t * dy;

        return std::hypot(p.getX() - projX, p.getY() - projY);
    };

    double minDist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < mBorders.size(); ++i)
    {
        const Point &a = mBorders[i];
        const Point &b = mBorders[(i + 1) % mBorders.size()];
        double dist = pointToSegmentDistance(p, a, b);
        minDist = std::min(minDist, dist);
    }

    return minDist;
}

/**
 * @brief Checks if the line segment between two points collides with any obstacle or border.
 *
 * This function interpolates points along the line segment between two given points
 * and checks if any of these points are too close to or inside an obstacle. The number
 * of interpolated points is determined based on the distance between the two points,
 * with a fixed sampling rate of 3 samples per unit of distance.
 *
 * @param p1 The starting point of the line segment.
 * @param p2 The ending point of the line segment.
 * @return true If the line segment collides with an obstacle or border.
 * @return false If the line segment does not collide with any obstacle or border.
 */
bool AStarGreedy::collidesWithObstacleOrBorder(const Point &p1, const Point &p2) const
{
    double distance = std::hypot(p2.getX() - p1.getX(), p2.getY() - p1.getY());

    // 3 samples per one unit of distance
    const double samplesPerUnit = 3.0;
    int numSamples = std::max(1, static_cast<int>(distance * samplesPerUnit));

    for (int i = 0; i <= numSamples; ++i)
    {
        double t = static_cast<double>(i) / numSamples;
        Point interp(
            p1.getX() + t * (p2.getX() - p1.getX()),
            p1.getY() + t * (p2.getY() - p1.getY()));

        for (const auto &obs : mObstacles)
        {
            if (obs.isTooCloseToObstacle(interp, minDistObs) || obs.isInsideObstacle(interp))
            {
                return true;
            }
        }
    }

    return false; // No collision detected
}

/**
 * @brief Estimates the time required to traverse a given path.
 * 
 * This function calculates the total time required to traverse a path
 * represented as a vector of points. The time is computed based on the
 * Euclidean distance between consecutive points in the path and a constant
 * velocity (VELOCITY).
 * 
 * @param path A vector of Point objects representing the path to be traversed.
 *             Each Point contains x and y coordinates.
 * @return The estimated time required to traverse the path. If the path
 *         contains fewer than two points, the function returns 0.0.
 */
double AStarGreedy::estimatePathTime(const std::vector<Point> &path) const
{
    if (path.size() < 2)
        return 0.0;

    double time = 0.0;

    for (size_t i = 1; i < path.size(); ++i)
    {
        double dx = path[i].getX() - path[i - 1].getX();
        double dy = path[i].getY() - path[i - 1].getY();
        double dist = std::hypot(dx, dy);
        time += dist / VELOCITY;
    }

    return time;
}

/**
 * @brief Builds the meta-graph for the A* Greedy algorithm.
 * 
 * This function constructs a meta-graph by calculating paths and costs between
 * key points, which include the start point, goal point, and all victim locations.
 * It uses the A* algorithm to find paths between these points and performs various
 * checks to ensure the validity of the paths.
 * 
 * The meta-graph is represented by two data structures:
 * - `mPaths`: A mapping of point pairs to their corresponding paths.
 * - `mCosts`: A mapping of point pairs to the cost of traveling between them.
 * 
 * The function performs the following steps:
 * 1. Initializes the key points (start, goal, and victims).
 * 2. Iterates over all pairs of key points to compute paths and costs.
 * 3. Validates each path for collisions with obstacles or borders.
 * 4. Ensures the estimated time for each path does not exceed the time limit.
 * 5. Stores valid paths and their costs in the meta-graph data structures.
 * 
 * @note Paths that are invalid, collide with obstacles, or exceed the time limit
 *       are skipped and not added to the meta-graph.
 * 
 * @warning This function assumes that the A* algorithm (`aStar`) and other helper
 *          functions (e.g., `isTooCloseToBorder`, `estimatePathTime`) are implemented
 *          and work correctly.
 * 
 * @details
 * - If a path is invalid or collides with an obstacle, a warning is printed to `std::cerr`.
 * - If a path's estimated time exceeds the time limit, it is skipped with a warning.
 * - The function outputs the number of paths and costs added to the meta-graph.
 */
void AStarGreedy::buildMetaGraph()
{
    /* mCosts.clear();
    mPaths.clear(); */

    std::vector<Point> keyPoints = {mStart, mGoal};
    for (const Victim &v : mVictims)
        keyPoints.emplace_back(v.x, v.y);

    auto collidesWithObstacle = [&](const std::vector<Point> &path)
    {
        for (const Point &p : path)
        {
            // Allow passing through the goal
            if (p == mGoal)
                continue;

            // Check collision with border
            if (isTooCloseToBorder(p, minDistBorder))
            {
                std::cerr << "Path collides with border at " << p.toString() << "\n";
                return true;
            }

            // Check collision with any obstacle
            for (const Obstacle &obs : mObstacles)
            {
                if (obs.isInsideObstacle(p) || obs.isTooCloseToObstacle(p, minDistObs))
                {
                    std::cerr << "Path collides with obstacle at " << p.toString() << "\n";
                    return true;
                }
            }
        }
        return false;
    };

    for (size_t i = 0; i < keyPoints.size(); ++i)
    {
        for (size_t j = i + 1; j < keyPoints.size(); ++j)
        {
            const Point &p1 = keyPoints[i];
            const Point &p2 = keyPoints[j];

            double cost;
            std::vector<Point> path = aStar(p1, p2, cost);

            if (path.empty() || path.back() != p2)
            {
                std::cerr << "Invalid path from " << p1.toString()
                          << " to " << p2.toString()
                          << ", ends at " << (path.empty() ? "empty" : path.back().toString()) << "\n";
                continue;
            }

            // further check the path itself for precision
            if (collidesWithObstacle(path))
                continue;

            double estimatedTime = estimatePathTime(path);
            if (estimatedTime > (mTimeLimit / VELOCITY))
            {
                std::cerr << "Skipping path " << p1.toString() << " → " << p2.toString()
                          << " because time " << estimatedTime << " > limit " << (mTimeLimit / VELOCITY) << "\n";
                continue;
            }
            std::cout << "Path from " << p1.toString() << " to " << p2.toString()
                      << " with cost " << cost << " and estimated time " << estimatedTime << "\n";

            mPaths[{p1, p2}] = path;
            mPaths[{p2, p1}] = std::vector<Point>(path.rbegin(), path.rend());

            mCosts[{p1, p2}] = cost;
            mCosts[{p2, p1}] = cost;
        }
    }

    std ::cout << "Meta graph built with " << mPaths.size() << " paths and " << mCosts.size() << " costs.\n";
}

/**
 * @brief Finds a victim at the specified point.
 * 
 * This function searches through the list of victims (`mVictims`) to find a victim
 * located at the given point `p`. If a victim is found at the specified point, 
 * a pointer to that victim is returned. If no victim is found, the function 
 * returns `nullptr`.
 * 
 * @param p The point to search for a victim.
 * @return const Victim* Pointer to the victim at the specified point, or `nullptr` if no victim is found.
 */
const Victim *AStarGreedy::findVictimAt(const Point &p) const
{
    for (const auto &v : mVictims)
        if (Point(v.x, v.y) == p)
            return &v;
    return nullptr;
}

/**
 * @brief Computes the total cost of traversing a given order of points.
 * 
 * This function calculates the total path cost for a sequence of points
 * (order) based on precomputed costs stored in the `mCosts` map. If a cost
 * between two consecutive points in the order is not found in the map, the
 * function returns infinity to indicate that the path is invalid or
 * incomplete.
 * 
 * @param order A vector of points representing the sequence of points to traverse.
 * @return The total cost of the path as a double. If any segment of the path
 *         is not found in the cost map, the function returns 
 *         `std::numeric_limits<double>::infinity()`.
 */
double AStarGreedy::pathCostForOrder(const std::vector<Point> &order)
{
    double totalCost = 0.0;
    for (size_t i = 0; i + 1 < order.size(); ++i)
    {
        if (mCosts.count({order[i], order[i + 1]}))
        {
            totalCost += mCosts[{order[i], order[i + 1]}];
        }
        else
        {
            return std::numeric_limits<double>::infinity();
        }
    }
    return totalCost;
}

/**
 * @brief Performs the 2-opt optimization algorithm to improve the order of points
 *        in a given path by iteratively reversing segments of the path to minimize
 *        the total path cost.
 * 
 * @param order A reference to a vector of Points representing the current order of the path.
 *              This vector will be modified in-place to reflect the optimized order.
 * 
 * The function uses a greedy approach to iteratively improve the path by checking all
 * possible segment reversals (2-opt swaps) and selecting the one that reduces the path cost.
 * The process continues until no further improvement can be made.
 * 
 * The path cost is calculated using the `pathCostForOrder` function, which is assumed
 * to compute the total cost of traversing the path in the given order.
 * 
 * Complexity:
 * - Worst-case time complexity: O(n^3), where n is the size of the `order` vector.
 *   This is due to the nested loops and the cost calculation for each potential swap.
 * 
 * Note:
 * - The function assumes that the `order` vector contains at least 3 points.
 * - The optimization process modifies the input vector directly.
 */
void AStarGreedy::twoOptOptimization(std::vector<Point> &order)
{
    bool improved = true;
    double bestCost = pathCostForOrder(order);
    while (improved)
    {
        improved = false;
        for (size_t i = 1; i < order.size() - 2; ++i)
        {
            for (size_t j = i + 1; j < order.size() - 1; ++j)
            {
                std::vector<Point> newOrder = order;
                std::reverse(newOrder.begin() + i, newOrder.begin() + j + 1);

                double newCost = pathCostForOrder(newOrder);
                if (newCost < bestCost)
                {
                    order = newOrder;
                    bestCost = newCost;
                    improved = true;
                    break;
                }
            }
            if (improved)
                break;
        }
    }
}

/**
 * @brief Computes the angle (in radians) between two vectors defined by three points.
 *
 * This function calculates the angle formed at point `b` by the vectors `ab` and `bc`.
 * The angle is computed using the dot product and the magnitudes of the vectors.
 *
 * @param a The starting point of the first vector.
 * @param b The common point where the two vectors meet.
 * @param c The ending point of the second vector.
 * @return The angle in radians between the two vectors. Returns 0 if either vector has zero length.
 *
 * @note The result is clamped to the range [0, π] to ensure numerical stability.
 */
double AStarGreedy::angleBetween(const Point &a, const Point &b, const Point &c)
{
    double abx = b.getX() - a.getX();
    double aby = b.getY() - a.getY();
    double bcx = c.getX() - b.getX();
    double bcy = c.getY() - b.getY();

    double ab_len = std::sqrt(abx * abx + aby * aby);
    double bc_len = std::sqrt(bcx * bcx + bcy * bcy);

    if (ab_len == 0 || bc_len == 0)
        return 0; // Degenerate case

    double cos_angle = (abx * bcx + aby * bcy) / (ab_len * bc_len);
    // Clamp cos_angle to [-1,1] to avoid domain errors in acos
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));

    double angle = std::acos(cos_angle); // radians
    return angle;
}

/**
 * @brief Executes the A* Greedy algorithm to find the best path and returns it.
 *
 * This function computes the best path using the A* Greedy algorithm, collects the total value
 * along the path, and appends the last point of the path to itself before returning the result.
 *
 * @param[out] totalValueCollected A reference to a double where the total value collected along the path will be stored.
 * @param[in] attempt The current attempt number, which may influence the pathfinding logic.
 * @return A vector of Points representing the best path found by the algorithm.
 */
std::vector<Point> AStarGreedy::run(double &totalValueCollected, int attempt)
{
    std::vector<Point> bestPath = findBestPath(totalValueCollected, attempt);

    bestPath.push_back(bestPath.back()); // Add the last point

    return bestPath;
}

// osnova
/**
 * @brief Finds the best path from the start point to the goal point while collecting victims based on a greedy algorithm.
 * 
 * This function attempts to find an optimal path that maximizes the value collected from victims 
 * while adhering to a time limit. It uses a greedy approach to select victims based on their 
 * value-to-cost ratio and applies penalties for proximity to borders and obstacles. The function 
 * also optimizes the order of visiting victims and constructs a detailed path.
 * 
 * @param[out] totalValueCollected The total value collected from victims along the path.
 * @param[in] attempt The current attempt number, used to adjust penalty multipliers.
 * 
 * @return A vector of Points representing the full path from the start to the goal, including 
 *         intermediate points for victims and optimized segments.
 * 
 * @details
 * - The function first checks if a direct path from the start to the goal exists and whether it 
 *   satisfies the time limit.
 * - Victims are sorted in descending order of their radius (value).
 * - A greedy algorithm is used to iteratively add victims to the path if they can be reached 
 *   within the remaining time and provide a good value-to-cost ratio.
 * - Penalties are applied for sharp turns, proximity to borders, and proximity to obstacles.
 * - If time allows, a 2-opt optimization is applied to improve the order of visiting victims.
 * - The final path is constructed by concatenating segments between consecutive points in the 
 *   optimized order.
 * - The total value collected is calculated based on the victims included in the path.
 * 
 * @note The function assumes that the costs and paths between points are precomputed and stored 
 *       in `mCosts` and `mPaths`, respectively.
 * 
 * @warning If no valid path exists between certain points, those segments are skipped, and the 
 *          function may return an incomplete path.
 */
std::vector<Point> AStarGreedy::findBestPath(double &totalValueCollected, int attempt)
{
    totalValueCollected = 0.0;

    // 1. Проверяем прямой путь от старта к цели
    if (!mCosts.count({mStart, mGoal}))
    {
        std::cerr << "Error: No path from start to goal!" << std::endl;
        return {};
    }

    double directCost = mCosts[{mStart, mGoal}];
    std::cout << "Direct path cost from start to goal: " << directCost << std::endl;
    if (directCost > mTimeLimit)
    {
        std::cerr << "Warning: Direct path from start to goal exceeds time limit!" << std::endl;
        return {};
    }

    // Начинаем формировать order — сначала старт и цель
    std::vector<Point> order = {mStart, mGoal};
    double totalCost = directCost;

    std::unordered_set<Point> visitedVictims;
    Point current = mStart;

    // I want to order mVictims from the biggest radius to the smallest
    std::sort(mVictims.begin(), mVictims.end(), [](const Victim &a, const Victim &b)
              {
                  return a.radius > b.radius;
              });
    std::cout << "Victims sorted by radius in descending order." << std::endl;

    double remainingTime = mTimeLimit;
    std::cout << "BEFORE: Remaining time: " << remainingTime << " seconds." << std::endl;

    // 2. Жадно добавляем жертв, если есть время
    while (true)
    {
        const Victim *bestVictim = nullptr;
        double bestRatio = -1;
        Point bestPoint;

        for (const auto &v : mVictims)
        {
            Point vp(v.x, v.y);

            if (visitedVictims.count(vp))
            {
                std::cout << "Victim at " << vp.toString() << " already visited, skipping." << std::endl;
                continue;
            }

            if (!mCosts.count({current, vp}) || !mCosts.count({vp, mGoal}))
            {
                std::cout << "Victim at " << vp.toString() << " doesnt have a path." << std::endl;
                continue;
            }

            double costToVictim = mCosts[{current, vp}];
            double costVictimToGoal = mCosts[{vp, mGoal}];
            double costCurrentToGoal = mCosts[{current, mGoal}];

            std::cout << "" << "Checking victim at " << vp.toString() << " | cost to victim: "
                      << costToVictim << ", cost from victim to goal: "
                      << costVictimToGoal << ", current to goal: "
                      << costCurrentToGoal
                      << ", total cost: " << totalCost
                      << ", remainingTime: " << remainingTime << std::endl;

            double newTotalCost = totalCost - costCurrentToGoal + costToVictim + costVictimToGoal;
            std::cout << "New total cost if added: " << newTotalCost << std::endl;

            if (newTotalCost >= mTimeLimit * VELOCITY)
            {
                std::cout << "Skipping victim at " << vp.toString()
                          << " because new total cost " << newTotalCost
                          << " exceeds remaining time " << remainingTime * VELOCITY
                          << " or is greater than current total cost " << totalCost << std::endl;
                continue;
            }

            // angle
            double turnAngle = angleBetween(bestPoint, current, vp); // в радианах

            // time penalty
            double timeFactor = 1.0;
            double penaltyMultiplier = (1.0 + 0.2 * std::min(attempt, 3)) * timeFactor;

            double alpha = penaltyMultiplier * 5 * VELOCITY;
            double penalty = alpha * turnAngle;

            if (isTooCloseToBorder(vp, 0.2f))
            {
                std::cerr << "Warning: Victim at " << vp.toString() << " is too close to the border!" << std::endl;
                continue;
            }

            // if too clode to the border, apply additional penalties
            double minBorderDist = distanceToClosestBorder(vp);
            if (minBorderDist < 0.6f)
            {
                penalty += penaltyMultiplier * 15 * VELOCITY * (0.6f * 2 - minBorderDist);
            }

            // if too close to the obstacles, apply additional penalties
            double minObsDist = std::numeric_limits<double>::max();
            for (const Obstacle &obs : mObstacles)
            {
                double d = obs.distanceTo(vp);
                minObsDist = std::min(minObsDist, d);
            }
            if (minObsDist < minDistObs)
            {
                penalty += penaltyMultiplier * 10 * VELOCITY * (minDistObs * 2 - minObsDist);
            }

            std::cout << "Check the victim at " << vp.toString()
                      << " | value: " << v.radius
                      << " | cost to victim: " << costToVictim
                      << ", cost from victim to goal: " << costVictimToGoal
                      << ", new total cost: " << newTotalCost
                      << ", turn angle: " << turnAngle
                      << ", penalty: " << penalty
                      << ", minObsDist: " << minObsDist
                      << ", minBorderDist: " << minBorderDist
                      << std::endl;

            // Жадный критерий
            double ratio = v.radius / (costToVictim + costVictimToGoal + penalty);
            
            if (ratio > bestRatio)
            {
                std::cout << "Found better victim at " << vp.toString()
                          << " with ratio: " << ratio
                          << " (previous best: " << bestRatio << ")" << std::endl;
                bestRatio = ratio;
                bestVictim = &v;
                bestPoint = vp;
            }
            std::cout << "-------------------------------------------------------------------/n" << std::endl;
        }

        if (!bestVictim)
            break;

        // Order перед целью
        std::cout << "ADD: Added victim at " << bestPoint.toString() << " with radius " << bestVictim->radius << std::endl;
        order.insert(order.end() - 1, bestPoint);
        visitedVictims.insert(bestPoint);

        // Пересчитываем текущий путь и стоимость
        totalCost = 0;
        for (size_t i = 0; i + 1 < order.size(); ++i)
        {
            totalCost += mCosts[{order[i], order[i + 1]}];
        }

        remainingTime = mTimeLimit - totalCost / VELOCITY;

        if (remainingTime <= 10)
        {
            std::cout << "No more time left to add victims, stopping." << std::endl;
            break;
        }
        
        std::cout << "ADD: New total cost after adding victim: " << totalCost << std::endl;
        std::cout << "ADD: Remaining time after adding victim: " << remainingTime << std::endl;

        std::cout << "////////////////////////////////////////////////////////////" << std::endl;
        std::cout << "////////////////////////////////////////////////////////////" << std::endl;

        current = order[order.size() - 2]; // last added victim
    }

    // 3. Optimization of the order
    if (order.size() > 2)
    {
        twoOptOptimization(order);
    }

    // 4. Full path from order from segments
    std::vector<Point> fullPath;
    std::unordered_set<Point> visitedNodes;
    fullPath.push_back(order[0]);
    visitedNodes.insert(order[0]);

    for (size_t i = 0; i + 1 < order.size(); ++i)
    {
        if (!mPaths.count({order[i], order[i + 1]}))
        {
            std::cerr << "No path between " << order[i].toString() << " and " << order[i + 1].toString() << std::endl;
            continue;
        }
        const std::vector<Point> &segment = mPaths[{order[i], order[i + 1]}];

        for (size_t j = 0; j < segment.size(); ++j)
        {
            if (fullPath.empty() || segment[j] != fullPath.back())
            {
                fullPath.push_back(segment[j]);
            }
        }
    }

    // 5. Calculate total value collected
    totalValueCollected = 0.0;
    for (const Point &p : order)
    {
        const Victim *v = findVictimAt(p);
        if (v)
            totalValueCollected += v->radius;
    }

    std::cout << "Total value collected: " << totalValueCollected << std::endl;
    std::cout << "Full path: ";
    
    for (const Point &p : fullPath)
    {
        std::cout << p.toString() << " ";
    }
    std::cout << std::endl;

    return fullPath;
}
