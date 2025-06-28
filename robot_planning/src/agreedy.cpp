#include "../include/robotPlanning/agreedy.hpp"
#include "../include/robotPlanning/multiPointMarkovDubins.hpp"
#include <limits>
#include <cmath>
#include <set>
#define VELOCITY 0.26
const float minDistObs = 0.2f; // Minimum distance to obstacles
const float minDistBorder = 0.2f; // Minimum distance to borders

AStarGreedy::AStarGreedy(Graph& graph, const std::vector<Victim>& victims,
                         const Point& start, const Point& goal, double timeLimit, std::vector<Obstacle> obstacles, std::vector<Point> borders)
    : mGraph(graph), mVictims(victims), mStart(start), mGoal(goal), mTimeLimit(timeLimit), 
      mObstacles(obstacles), mBorders(borders) 
{
    buildMetaGraph();
    std::cout << "CONSTRUCTOR ASTARGREEDY\n";
    std::cout << "Start: " << mStart.toString() << ", Goal: " << mGoal.toString() << "\n";
    std::cout << "Meta graph built with " << mPaths.size() << " paths.\n";
    std::cout << "Total costs calculated: " << mCosts.size() << "\n";
    if (mPaths.empty() || mCosts.empty()) {
        std::cerr << "Error: No paths or costs calculated in the meta graph.\n";
    }
    std::cout << "AStarGreedy initialized with time limit: " << mTimeLimit << "\n";
    std::cout << "CONSTRUCTOR ASTARGREEDY CREATED\n";
}

double AStarGreedy::heuristic(const Point& a, const Point& b) {
    double dx = a.getX() - b.getX();
    double dy = a.getY() - b.getY();
    return std::sqrt(dx * dx + dy * dy);
}

/* double AStarGreedy::heuristic(const Point& a, const Point& b) {
    double th0 = 0.0; // начальный угол — можно зафиксировать или угадать
    double thf = 0.0; // конечный угол

    double sampleStep = 0.2; // дискретизация (шаг)
    curve dub = dubinsShortestPath(
        a.getX(), a.getY(), th0,
        b.getX(), b.getY(), thf,
        mObstacles,
        sampleStep,
        mBorders
    );

    if (!dub.values.totalLength()) {
        // На всякий случай fallback на евклидову, если путь не построен
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return std::sqrt(dx * dx + dy * dy);
    }

    return dub.values.totalLength();
}*/

Point AStarGreedy::findClosestGraphNode(const Point& query) const{
    double minDist = std::numeric_limits<double>::infinity();
    Point closest;

    for (const auto& entry : mGraph.getVertices()) {  // getVertices() — множество всех точек графа
        double dist = query.computeEuclideanDistance(entry);
        if (dist < minDist) {
            minDist = dist;
            closest = entry;
        }
    }

    //print
    /* std::cout << "Closest graph node to " << query.toString() 
              << " is " << closest.toString() 
              << " with distance " << minDist << "\n"; */

    return closest;
}

void AStarGreedy::addVictimPenaltyFromSegment(const Point& rawP1, const Point& rawP2, std::vector<Point>& visitedVictims)
{
    // Distance threshold: how close a victim should be to be affected
    constexpr double victimProximityThreshold = 1.0;

    auto pointToSegmentDistance = [](const Point& p, const Point& a, const Point& b) {
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

    for (const Victim& v : mVictims)
    {   
        Point vp(v.x, v.y);

        double dist = pointToSegmentDistance(vp, rawP1, rawP2);

        if (dist < victimProximityThreshold)
        {
            if (std::find(visitedVictims.begin(), visitedVictims.end(), vp) == visitedVictims.end())
            {
                visitedVictims.push_back(vp);
                /* std::cout << "Victim at " << vp.toString() 
                          << " is within proximity of segment (" 
                          << rawP1.toString() << ", " 
                          << rawP2.toString() << ") with distance " 
                          << dist << "\n"; */
            }

        }
    }

}

void AStarGreedy::addVictimPenalty(const Point& rawP, double penalty) 
{
    slowVictimPenalties_[rawP] += penalty;  // Добавляем штраф для жертвы
}




void AStarGreedy::addEdgePenaltyClosest(const Point& rawP1, const Point& rawP2, double penalty) {
    // Найти ближайшие точки в графе
    Point p1 = findClosestGraphNode(rawP1);
    Point p2 = findClosestGraphNode(rawP2);

    // Применить штраф
    mEdgePenalties[{p1, p2}] += penalty;
    mEdgePenalties[{p2, p1}] += penalty;  // если граф неориентированный

    /* std::cout << "Added penalty " << penalty 
              << " for nearest edge (" << p1.toString() 
              << ", " << p2.toString() << ")\n"; */
}

std::vector<Point> AStarGreedy::aStar(const Point& start, const Point& goal, double& pathCost) {
    std::unordered_map<Point, double> gScore;
    std::unordered_map<Point, Point> cameFrom;
    std::priority_queue<Node> openSet;

    gScore[start] = 0.0;
    Node startNode{start, 0.0, heuristic(start, goal), 0.0, {}};
    openSet.push(startNode);

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        if (current.point == goal) {
            std::vector<Point> path;
            Point p = goal;
            while (p != start) {
                path.push_back(p);
                p = cameFrom[p];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            pathCost = current.gCost;
            return path;
        }

        for (const Point& neighbor : mGraph.getEdge(current.point)) 
        {
            // Базовая стоимость перехода
            double baseCost = (2.5 * VELOCITY * heuristic(current.point, neighbor));

            // Добавляем штраф, если есть для ребра (current.point, neighbor)
            double penaltyEdge = 0.0;
            auto it = mEdgePenalties.find({current.point, neighbor});
            if (it != mEdgePenalties.end()) 
            {
                //print
                //std::cout << "Applying penalty for edge (" << current.point.toString() << ", " << neighbor.toString() << "): " << it->second << "\n";
                penaltyEdge = it->second; // например 1000.0
            }

            // Штрафы на жертву (т.е. на вершину neighbor)
            double penaltyVictim = 0.0;
            auto itVictim = slowVictimPenalties_.find(neighbor);
            if (itVictim != slowVictimPenalties_.end())
            {
                //std::cout << "Applying penalty for victim at " << neighbor.toString() << ": " << itVictim->second << "\n";
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

bool AStarGreedy::isTooCloseToBorder(const Point& p, double margin) const {
    if (mBorders.size() < 2) return true;  // Not initialized, be conservative

    auto pointToSegmentDistance = [](const Point& p, const Point& a, const Point& b) {
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

    for (size_t i = 0; i < mBorders.size(); ++i) {
        const Point& a = mBorders[i];
        const Point& b = mBorders[(i + 1) % mBorders.size()];
        double dist = pointToSegmentDistance(p, a, b);
        if (dist < margin) {
            return true;
        }
    }

    return false;
}

double AStarGreedy::distanceToClosestBorder(const Point& p) const {
    if (mBorders.size() < 2) return 0.0;  // Not initialized or malformed

    auto pointToSegmentDistance = [](const Point& p, const Point& a, const Point& b) {
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

    for (size_t i = 0; i < mBorders.size(); ++i) {
        const Point& a = mBorders[i];
        const Point& b = mBorders[(i + 1) % mBorders.size()];
        double dist = pointToSegmentDistance(p, a, b);
        minDist = std::min(minDist, dist);
    }

    return minDist;
}


bool AStarGreedy::collidesWithObstacleOrBorder(const Point& p1, const Point& p2) const 
{
    double distance = std::hypot(p2.getX() - p1.getX(), p2.getY() - p1.getY());

    // 3 samples per one unit of distance
    const double samplesPerUnit = 3.0; 
    int numSamples = std::max(1, static_cast<int>(distance * samplesPerUnit));

    for (int i = 0; i <= numSamples; ++i) {
        double t = static_cast<double>(i) / numSamples;
        Point interp(
            p1.getX() + t * (p2.getX() - p1.getX()),
            p1.getY() + t * (p2.getY() - p1.getY())
        );

        for (const auto& obs : mObstacles) {
            if (obs.isTooCloseToObstacle(interp, minDistObs) || obs.isInsideObstacle(interp)) {
                return true;
            }
        }
    }

    return false; // No collision detected
}

double AStarGreedy::estimatePathTime(const std::vector<Point>& path) const {
    if (path.size() < 2) return 0.0;

    double time = 0.0;

    for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].getX() - path[i - 1].getX();
        double dy = path[i].getY() - path[i - 1].getY();
        double dist = std::hypot(dx, dy);
        time += dist / VELOCITY;
    }

    return time;
}



void AStarGreedy::buildMetaGraph() {
    mCosts.clear();
    mPaths.clear();

    std::vector<Point> keyPoints = {mStart, mGoal};
    for (const Victim& v : mVictims)
        keyPoints.emplace_back(v.x, v.y);

    auto collidesWithObstacle = [&](const std::vector<Point>& path) 
    {
        for (const Point& p : path) {
            // Allow passing through the goal
            if (p == mGoal)
                continue;

            // Check collision with border
            if (isTooCloseToBorder(p, minDistBorder)) {
                std::cerr << "Path collides with border at " << p.toString() << "\n";
                return true;
            }
    
            // Check collision with any obstacle
            for (const Obstacle& obs : mObstacles) {
                if (obs.isInsideObstacle(p) || obs.isTooCloseToObstacle(p, minDistObs)) {
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
            const Point& p1 = keyPoints[i];
            const Point& p2 = keyPoints[j];

            double cost;
            std::vector<Point> path = aStar(p1, p2, cost);

            if (path.empty() || path.back() != p2) {
                std::cerr << "Invalid path from " << p1.toString()
                        << " to " << p2.toString()
                        << ", ends at " << (path.empty() ? "empty" : path.back().toString()) << "\n";
                continue;
            }

            // Optional: further check the path itself for precision
            if (collidesWithObstacle(path)) continue;

            double estimatedTime = estimatePathTime(path);
            if (estimatedTime > 60) {
                std::cerr << "Skipping path " << p1.toString() << " → " << p2.toString()
                        << " because time " << estimatedTime << " > limit " << 60 << "\n";
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

const Victim* AStarGreedy::findVictimAt(const Point& p) const {
    for (const auto& v : mVictims)
        if (Point(v.x, v.y) == p) return &v;
    return nullptr;
}

double AStarGreedy::pathCostForOrder(const std::vector<Point>& order) {
    double totalCost = 0.0;
    for (size_t i = 0; i + 1 < order.size(); ++i) {
        if (mCosts.count({order[i], order[i+1]})) {
            totalCost += mCosts[{order[i], order[i+1]}];
        } else {
            return std::numeric_limits<double>::infinity();
        }
    }
    return totalCost;
}

void AStarGreedy::twoOptOptimization(std::vector<Point>& order) {
    bool improved = true;
    double bestCost = pathCostForOrder(order);
    while (improved) {
        improved = false;
        for (size_t i = 1; i < order.size() - 2; ++i) {
            for (size_t j = i + 1; j < order.size() - 1; ++j) {
                std::vector<Point> newOrder = order;
                std::reverse(newOrder.begin() + i, newOrder.begin() + j + 1);

                double newCost = pathCostForOrder(newOrder);
                if (newCost < bestCost) {
                    order = newOrder;
                    bestCost = newCost;
                    improved = true;
                    break;  // optional: exit inner loop on improvement
                }
            }
            if (improved) break;  // optional: exit outer loop too
        }
    }
}

double AStarGreedy::angleBetween(const Point& a, const Point& b, const Point& c) {
    double abx = b.getX() - a.getX();
    double aby = b.getY() - a.getY();
    double bcx = c.getX() - b.getX();
    double bcy = c.getY() - b.getY();

    double ab_len = std::sqrt(abx * abx + aby * aby);
    double bc_len = std::sqrt(bcx * bcx + bcy * bcy);

    if (ab_len == 0 || bc_len == 0) return 0; // Degenerate case

    double cos_angle = (abx * bcx + aby * bcy) / (ab_len * bc_len);
    // Clamp cos_angle to [-1,1] to avoid domain errors in acos
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));

    double angle = std::acos(cos_angle); // radians
    return angle;
}

std::vector<Point> AStarGreedy::run(double& totalValueCollected, int attempt) 
{
    std::vector<Point> bestPath = findBestPath(totalValueCollected, attempt);

    bestPath.push_back(bestPath.back()); // Add the last point

    return bestPath;

}

// osnova
std::vector<Point> AStarGreedy::findBestPath(double& totalValueCollected, int attempt) {
    totalValueCollected = 0.0;

    // 1. Проверяем прямой путь от старта к цели
    if (!mCosts.count({mStart, mGoal})) {
        std::cerr << "Error: No path from start to goal!" << std::endl;
        return {};
    }

    double directCost = mCosts[{mStart, mGoal}];
    std::cout << "Direct path cost from start to goal: " << directCost << std::endl;
    if (directCost > mTimeLimit) {
        std::cerr << "Warning: Direct path from start to goal exceeds time limit!" << std::endl;
        return {};
    }

    // Начинаем формировать order — сначала старт и цель
    std::vector<Point> order = {mStart, mGoal};
    double totalCost = directCost;

    std::unordered_set<Point> visitedVictims;
    Point current = mStart;

    // i want to rder mVictims from the biggest radius to the smallest
    std::sort(mVictims.begin(), mVictims.end(), [](const Victim& a, const Victim& b) {
        return a.radius > b.radius; // Сортируем по убыванию радиуса
    });
    std::cout << "Victims sorted by radius in descending order." << std::endl;

    // 2. Жадно добавляем жертв, если есть время
    while (true) {
        const Victim* bestVictim = nullptr;
        double bestRatio = -1;
        Point bestPoint;

        double remainingTime = mTimeLimit;

        for (const auto& v : mVictims) {
            //print victim
            //std::cout << "Evaluating victim at (" << v.x << ", " << v.y << ") with radius " << v.radius << std::endl;

            Point vp(v.x, v.y);

            if (visitedVictims.count(vp)){
                std::cout << "Victim at " << vp.toString() << " already visited, skipping." << std::endl;
                continue;
            } 

            if (!mCosts.count({current, vp}) || !mCosts.count({vp, mGoal})){ 
                //std::cout << "Victim at " << vp.toString() << " doesnt have a path." << std::endl;
                continue;
            }

            double costToVictim = mCosts[{current, vp}];
            double costVictimToGoal = mCosts[{vp, mGoal}];
            double costCurrentToGoal = mCosts[{current, mGoal}];

            if ((costToVictim) > costCurrentToGoal) {
                //std::cout << "Victim at " << vp.toString() << " is further than current to goal, skipping." << std::endl;
                // Жертва дальше ворот — пропускаем
                continue;
            }

            // Проверяем, что суммарное время с добавлением жертвы не превышает лимит
            std::cout << "" << "Checking victim at " << vp.toString() << " | cost to victim: " 
                      << costToVictim << ", cost from victim to goal: " 
                      << costVictimToGoal << ", current to goal: " 
                      << costCurrentToGoal << std::endl;

            double newTotalCost = totalCost - costCurrentToGoal + costToVictim + costVictimToGoal;
            std::cout << "New total cost if added: " << newTotalCost << std::endl;
            if (newTotalCost >= remainingTime)
            {
                std::cout << "Adding victim at " << vp.toString() 
                          << " would exceed remaining time, skipping." << std::endl;
                continue;
            }

            // Расчет угла поворота
            double turnAngle = angleBetween(bestPoint, current, vp); // в радианах

            // Коэффициенты штрафа за поворот
            double alpha = 5* VELOCITY ; // подбирай экспериментально
            double penalty = alpha * turnAngle;
 
            if(isTooCloseToBorder(vp, 0.2f))
            {
                std::cerr << "Warning: Victim at " << vp.toString() << " is too close to the border!" << std::endl;
                continue; // Пропускаем жертву, если она слишком близко к границе
            } 

            // if too clode to the border, apply additional penalties
            double minBorderDist = distanceToClosestBorder(vp);
            if (minBorderDist < 0.6f) {
                penalty += 15* VELOCITY * (0.6f*2 - minBorderDist); // штраф за близость к границе
                //std::cout << "Penalty for being too close to border: " << minBorderDist << std::endl;
            }

            //if too close to the obstacles, apply additional penalties
            double minObsDist = std::numeric_limits<double>::max();
            for (const Obstacle& obs : mObstacles) {
                double d = obs.distanceTo(vp);  
                minObsDist = std::min(minObsDist, d);
            }
            if (minObsDist < minDistObs) {
                penalty +=  10* VELOCITY  * (minDistObs*2 - minObsDist); // штраф за близость к препятствиям
                //std::cout << "Penalty for being too close to obstacles: " << minObsDist << std::endl;
            }

            // Выводим информацию о жертве
            std::cout << "Evaluating victim at " << vp.toString() 
                      << " | value: " << v.radius
                      << " | cost to victim: " << costToVictim 
                      << ", cost from victim to goal: " << costVictimToGoal 
                      << ", new total cost: " << newTotalCost 
                      << ", turn angle: " << turnAngle 
                      << ", penalty: " << penalty 
                      << ", minObsDist: " << minObsDist
                      << ", minBorderDist: " << minBorderDist
                      << std::endl;

            // Жадный критерий (например, радиус / времени)
            double ratio = v.radius / (costToVictim + costVictimToGoal + penalty);
            if (ratio > bestRatio) {
                bestRatio = ratio;
                bestVictim = &v;
                bestPoint = vp;
            }
            std::cout << "-------------------------------------------------------------------" << std::endl;
        }

        if (!bestVictim) break;

        double costToVictim = mCosts[{current, bestPoint}];
        double costVictimToGoal = mCosts[{bestPoint, mGoal}];
        double costCurrentToGoal = mCosts[{current, mGoal}];
        double addedCost = costToVictim + costVictimToGoal - costCurrentToGoal;

        remainingTime -= addedCost;
        if (remainingTime < 0) {
            std::cout << "No more time left to add victims, stopping." << std::endl;
            break; // Нет времени на добавление жертв
        }

        // Вставляем жертву в order перед целью
        // print what we add
        std::cout << "Adding victim at " << bestPoint.toString() << " with radius " << bestVictim->radius << std::endl;
        order.insert(order.end() - 1, bestPoint);
        visitedVictims.insert(bestPoint);

        // Пересчитываем текущий путь и стоимость
        totalCost = 0;
        for (size_t i = 0; i + 1 < order.size(); ++i) {
            totalCost += mCosts[{order[i], order[i+1]}];
        }
        current = order[order.size() - 2];  // предпоследняя точка — последняя добавленная жертва
    }

    // 3. Оптимизируем порядок посещения жертв (без затрагивания старта и цели)
    if (order.size() > 2) {
        twoOptOptimization(order);
    }
 
    // 4. Строим полный детальный путь по order из сегментов
    std::vector<Point> fullPath;
    std::unordered_set<Point> visitedNodes;
    fullPath.push_back(order[0]);
    visitedNodes.insert(order[0]);

    for (size_t i = 0; i + 1 < order.size(); ++i) {
        if (!mPaths.count({order[i], order[i+1]})) {
            std::cerr << "No path between " << order[i].toString() << " and " << order[i+1].toString() << std::endl;
            continue;
        }
        const std::vector<Point>& segment = mPaths[{order[i], order[i+1]}];

        for (size_t j = 0; j < segment.size(); ++j) {
            if (fullPath.empty() || segment[j] != fullPath.back()) {
                fullPath.push_back(segment[j]);
            }
        }

    } 

    // Считаем итоговое значение собранных жертв по order (кроме старта и цели)
    totalValueCollected = 0.0;
    for (const Point& p : order) {
        const Victim* v = findVictimAt(p);
        if (v) totalValueCollected += v->radius; // или v->value, если есть
    }

    std::cout << "Total value collected: " << totalValueCollected << std::endl;

    // Выводим полный путь
    std::cout << "Full path: ";
    for (const Point& p : fullPath) {
        std::cout << p.toString() << " ";
    }
    std::cout << std::endl;

    return fullPath;

}
 
/* std::vector<Point> AStarGreedy::findBestPath(double& totalValueCollected, int attempt) {
    std::vector<PathOption> candidates;

    for (int k = 0; k < 10 * (attempt + 1); ++k) {
        std::vector<Point> order = {mStart, mGoal};
        std::unordered_set<Point> visited;
        double totalCost = mCosts[{mStart, mGoal}];
        double remainingTime = mTimeLimit;

        Point current = mStart;
        double totalValue = 0;

        while (true) {
            std::vector<std::pair<double, const Victim*>> ranked;

            for (const auto& v : mVictims) {
                Point vp(v.x, v.y);
                if (visited.count(vp)) continue;
                if (!mCosts.count({current, vp}) || !mCosts.count({vp, mGoal})) continue;

                double toV = mCosts[{current, vp}];
                double vToGoal = mCosts[{vp, mGoal}];
                double oldToGoal = mCosts[{current, mGoal}];
                double newCost = totalCost - oldToGoal + toV + vToGoal;

                if (newCost >= remainingTime) continue;

                double angle = angleBetween(order.size() > 1 ? order[order.size() - 2] : current, current, vp);
                double penalty = 5.0 * angle;
                std::cout << "Angle penalty for victim at " << vp.toString() << ": " << penalty << "\n";

                // Additional penalty: proximity to obstacles and borders
                double minObsDist = std::numeric_limits<double>::max();
                double minBorderDist = std::numeric_limits<double>::max();

                // Check distance to each obstacle
                for (const Obstacle& obs : mObstacles) {
                    double d = obs.distanceTo(vp);  
                    minObsDist = std::min(minObsDist, d);
                }

                // Check distance to each border point
                for (const Point& bp : mBorders) {
                    double dx = vp.getX() - bp.getX();
                    double dy = vp.getY() - bp.getY();
                    double d = std::sqrt(dx * dx + dy * dy);
                    minBorderDist = std::min(minBorderDist, d);
                }

                // Apply penalty if too close
                if (minObsDist < 0.3) penalty += 10.0 * (0.5 - minObsDist);
                if (minBorderDist < 0.3) penalty += 10.0 * (0.5 - minBorderDist);

                std::cout << "Distance to obstacles: " << minObsDist << ", to borders: " << minBorderDist << "\n";

                double ratio = v.radius / (toV + vToGoal + penalty);
                ranked.emplace_back(ratio, &v);
            }

            if (ranked.empty()) break;
            std::sort(ranked.begin(), ranked.end(), [](auto& a, auto& b) { return a.first > b.first; });

            int victimIdx = k % ranked.size();  // sample different ones based on k
            const Victim* chosen = ranked[victimIdx].second;
            Point chosenP(chosen->x, chosen->y);

            order.insert(order.end() - 1, chosenP);
            visited.insert(chosenP);
            totalValue += chosen->radius;

            totalCost = 0.0;
            for (size_t i = 0; i + 1 < order.size(); ++i)
                totalCost += mCosts[{order[i], order[i+1]}];
            current = chosenP;
        }

        if (order.size() > 2) twoOptOptimization(order);

        double finalCost = pathCostForOrder(order);
        if (finalCost <= mTimeLimit)
            candidates.push_back({order, finalCost, totalValue, totalValue / finalCost});
    }

    // Sort by score
    std::sort(candidates.begin(), candidates.end(), [](const PathOption& a, const PathOption& b) {
        return a.score > b.score;
    });

    if (attempt >= candidates.size()) {
        std::cerr << "No path found for attempt #" << attempt << "\n";
        return {};
    }

    const auto& selected = candidates[attempt];
    totalValueCollected = selected.totalValue;

    // Now build the detailed path from mPaths
    std::vector<Point> fullPath;
    fullPath.push_back(selected.order[0]);

    for (size_t i = 0; i + 1 < selected.order.size(); ++i) {
        const auto& segment = mPaths[{selected.order[i], selected.order[i+1]}];
        for (const auto& pt : segment)
            if (fullPath.empty() || pt != fullPath.back())
                fullPath.push_back(pt);
    }

    return fullPath;
} */