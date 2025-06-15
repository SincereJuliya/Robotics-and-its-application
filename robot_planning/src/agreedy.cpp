#include "../include/robotPlanning/agreedy.hpp"
#include <limits>
#include <cmath>
#include <set>

AStarGreedy::AStarGreedy(Graph& graph, const std::vector<Victim>& victims,
                         const Point& start, const Point& goal, double timeLimit)
    : mGraph(graph), mVictims(victims), mStart(start), mGoal(goal), mTimeLimit(timeLimit) {
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

        for (const Point& neighbor : mGraph.getEdge(current.point)) {
            double tentativeG = gScore[current.point] + heuristic(current.point, neighbor);
            if (!gScore.count(neighbor) || tentativeG < gScore[neighbor]) {
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

void AStarGreedy::buildMetaGraph() {
    std::vector<Point> keyPoints = {mStart, mGoal};
    for (const Victim& v : mVictims)
        keyPoints.push_back(Point(v.x, v.y));

    for (size_t i = 0; i < keyPoints.size(); ++i) {
        for (size_t j = 0; j < keyPoints.size(); ++j) {
            if (i == j) continue;
            double cost;
            std::vector<Point> path = aStar(keyPoints[i], keyPoints[j], cost);

            if (!path.empty()) {
                if (path.back() != keyPoints[j]) {
                    std::cerr << "Warning: path from " << keyPoints[i].toString()
                              << " does not reach " << keyPoints[j].toString()
                              << " but ends at " << path.back().toString() << "\n";
                    continue; 
                }
                mPaths[{keyPoints[i], keyPoints[j]}] = path;
                mCosts[{keyPoints[i], keyPoints[j]}] = cost;
            }
        }
    }
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
    while (improved) {
        improved = false;
        for (size_t i = 1; i < order.size() - 2; ++i) {
            for (size_t j = i + 1; j < order.size() - 1; ++j) {
                std::vector<Point> newOrder = order;
                std::reverse(newOrder.begin() + i, newOrder.begin() + j + 1);
                if (pathCostForOrder(newOrder) < pathCostForOrder(order)) {
                    order = newOrder;
                    improved = true;
                }
            }
        }
    }
}

/*
std::vector<Point> AStarGreedy::findBestPath(double& totalValueCollected) {
    totalValueCollected = 0.0;

    if (!mCosts.count({mStart, mGoal})) {
        std::cerr << "Error: No path from start to goal!" << std::endl;
        return {};
    }

    double directCost = mCosts[{mStart, mGoal}];
    if (directCost > mTimeLimit) {
        std::cerr << "Warning: Direct path from start to goal exceeds time limit!" << std::endl;
        return {};
    }

    std::vector<Point> order = {mStart, mGoal};

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

        if (segment.empty()) {
            std::cerr << "Empty segment from " << order[i].toString() << " to " << order[i+1].toString() << "\n";
            continue;
        }

        if (segment.back() != order[i+1]) {
            std::cerr << "Segment from " << order[i].toString() << " to " << order[i+1].toString()
                      << " ends at " << segment.back().toString() << ", not at goal!\n";
        }

        for (size_t j = 1; j < segment.size(); ++j) {
            if (visitedNodes.insert(segment[j]).second) {
                fullPath.push_back(segment[j]);
            }
        }
    }

    std::cout << "Full path: ";
    for (const Point& p : fullPath) {
        std::cout << p.toString() << " ";
    }
    std::cout << std::endl;

    return fullPath;
} */

double AStarGreedy::angleBetween(const Point& a, const Point& b, const Point& c) {
    double abx = b.getX() - a.getX();
    double aby = b.getY() - a.getY();
    double bcx = c.getX() - b.getX();
    double bcy = c.getY() - b.getY();

    double ab_len = std::sqrt(abx*abx + aby*aby);
    double bc_len = std::sqrt(bcx*bcx + bcy*bcy);

    if (ab_len == 0 || bc_len == 0) return 0;

    double cos_angle = (abx*bcx + aby*bcy) / (ab_len * bc_len);
    if (cos_angle > 1.0) cos_angle = 1.0;
    if (cos_angle < -1.0) cos_angle = -1.0;

    double angle = std::acos(cos_angle); // радианы
    return angle; // возвращаем в радианах, можно умножить на 180/π если нужно градусы
}

// osnova
std::vector<Point> AStarGreedy::findBestPath(double& totalValueCollected) {
    totalValueCollected = 0.0;

    // 1. Проверяем прямой путь от старта к цели
    if (!mCosts.count({mStart, mGoal})) {
        std::cerr << "Error: No path from start to goal!" << std::endl;
        return {};
    }

    double directCost = mCosts[{mStart, mGoal}];
    if (directCost > mTimeLimit) {
        std::cerr << "Warning: Direct path from start to goal exceeds time limit!" << std::endl;
        return {};
    }

    // Начинаем формировать order — сначала старт и цель
    std::vector<Point> order = {mStart, mGoal};
    double totalCost = directCost;

    std::unordered_set<Point> visitedVictims;
    Point current = mStart;

    // 2. Жадно добавляем жертв, если есть время
    while (true) {
        const Victim* bestVictim = nullptr;
        double bestRatio = -1;
        Point bestPoint;

        for (const auto& v : mVictims) {
            Point vp(v.x, v.y);
            if (visitedVictims.count(vp)) continue;

            // Проверяем приоритет ворот: если жертва дальше ворот — пропускаем
            if (!mCosts.count({current, vp}) || !mCosts.count({vp, mGoal})) continue;

            double costToVictim = mCosts[{current, vp}];
            double costVictimToGoal = mCosts[{vp, mGoal}];
            double costCurrentToGoal = mCosts[{current, mGoal}];

            if (costToVictim > costCurrentToGoal) {
                // Жертва дальше ворот — пропускаем
                continue;
            }

            // Проверяем, что суммарное время с добавлением жертвы не превышает лимит
            double newTotalCost = totalCost - costCurrentToGoal + costToVictim + costVictimToGoal;
            if (newTotalCost > mTimeLimit) continue;

            // Расчет угла поворота
            double turnAngle = angleBetween(bestPoint, current, vp); // в радианах

            // Коэффициенты штрафа за поворот
            double alpha = 6.0; // подбирай экспериментально
            double turnPenalty = alpha * turnAngle;

            std::cout << "Victim at " << vp.toString() << " | angle: " << turnAngle << " | penalty: " << (alpha * turnAngle) << std::endl;

            // Жадный критерий (например, радиус / времени)
            double ratio = v.radius / (costToVictim + costVictimToGoal + turnPenalty);
            if (ratio > bestRatio) {
                bestRatio = ratio;
                bestVictim = &v;
                bestPoint = vp;
            }
        }

        if (!bestVictim) break;

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
        /* // Добавляем сегмент, избегая дублирования точек
        for (size_t j = 1; j < segment.size(); ++j) {
            if (visitedNodes.insert(segment[j]).second) {
                fullPath.push_back(segment[j]);
            }
        } */
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

/* 
std::vector<Point> AStarGreedy::findBestPath(double& totalValueCollected) {
    Point current = mStart;
    std::unordered_set<Point> visitedVictims;

    std::vector<Point> order;  // Порядок посещения ключевых точек
    order.push_back(mStart);

    totalValueCollected = 0.0;
    double totalCost = 0.0;

    // Жадный выбор жертв — только ключевые точки, без построения полного пути
    while (true) {
        const Victim* bestVictim = nullptr;
        double bestRatio = -1;
        Point bestPoint;

        for (const auto& v : mVictims) {
            Point vp(v.x, v.y);
            if (visitedVictims.count(vp)) continue;

            double cost = mCosts.count({current, vp}) ? mCosts[{current, vp}] : std::numeric_limits<double>::infinity();
            double toGoal = mCosts.count({vp, mGoal}) ? mCosts[{vp, mGoal}] : std::numeric_limits<double>::infinity();

            if (totalCost + cost + toGoal > mTimeLimit) continue;

            double ratio = v.radius / (cost + toGoal + totalCost);
            if (ratio > bestRatio) {
                bestRatio = ratio;
                bestVictim = &v;
                bestPoint = vp;
            }
        }

        if (!bestVictim) break;

        double costToBest = mCosts[{current, bestPoint}];
        totalCost += costToBest;
        totalValueCollected += bestVictim->radius;
        visitedVictims.insert(bestPoint);
        order.push_back(bestPoint);
        current = bestPoint;
    }

    order.push_back(mGoal);

    // Оптимизируем порядок посещения жертв
    twoOptOptimization(order);

    // Строим полный путь из детальных сегментов по оптимизированному порядку
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
        for (size_t j = 1; j < segment.size(); ++j) {
            if (visitedNodes.insert(segment[j]).second) {
                fullPath.push_back(segment[j]);
            }
        }
    }

    return fullPath;
} */


/*
std::vector<Point> AStarGreedy::findBestPath(double& totalValueCollected) {
    Point current = mStart;
    std::unordered_set<Point> visitedVictims;
    //
    std::unordered_set<Point> visitedNodes;

    std::vector<Point> fullPath;
    double totalCost = 0.0;
    totalValueCollected = 0.0;

    fullPath.push_back(current);
    visitedNodes.insert(current);  // отметим стартовую точку как посещённую

    while (true) {
        const Victim* bestVictim = nullptr;
        double bestRatio = -1;
        double bestCost = 0;
        Point bestPoint;

        for (const auto& v : mVictims) {
            Point vp(v.x, v.y);
            if (visitedVictims.count(vp)) continue;

            double cost = mCosts.count({current, vp}) ? mCosts[{current, vp}] : std::numeric_limits<double>::infinity();
            double toGoal = mCosts.count({vp, mGoal}) ? mCosts[{vp, mGoal}] : std::numeric_limits<double>::infinity();

            if (totalCost + cost + toGoal > mTimeLimit) continue;

            double ratio = v.radius / (cost + toGoal); // вместо v.radius / cost
            if (ratio > bestRatio) {
                bestRatio = ratio;
                bestVictim = &v;
                bestPoint = vp;
                bestCost = cost;
            }
        }

        if (!bestVictim) break;

        std::vector<Point> pathToVictim = mPaths[{current, bestPoint}];
        pathToVictim.erase(pathToVictim.begin());  // avoid duplicating current
        fullPath.insert(fullPath.end(), pathToVictim.begin(), pathToVictim.end());

        totalCost += bestCost;
        totalValueCollected += bestVictim->radius;
        visitedVictims.insert(bestPoint);
        current = bestPoint;
    }

    // Add final path to goal
    //if (mPaths.count({current, mGoal})) {
    //    std::vector<Point> pathToGoal = mPaths[{current, mGoal}];
    //    pathToGoal.erase(pathToGoal.begin());
     //   fullPath.insert(fullPath.end(), pathToGoal.begin(), pathToGoal.end());
    //}
 

    // Добавить финальный путь до цели
    if (mPaths.count({current, mGoal})) {
        std::vector<Point> pathToGoal = mPaths[{current, mGoal}];
        for (size_t i = 1; i < pathToGoal.size(); ++i) {
            if (visitedNodes.insert(pathToGoal[i]).second)
                fullPath.push_back(pathToGoal[i]);
        }
    } else {
        std::cerr << "Warning: No path from " << current.toString() << " to goal. Attempting direct A*...\n";
        double fallbackCost;
        auto fallbackPath = aStar(current, mGoal, fallbackCost);
        if (!fallbackPath.empty()) {
            for (size_t i = 1; i < fallbackPath.size(); ++i) {
                if (visitedNodes.insert(fallbackPath[i]).second)
                    fullPath.push_back(fallbackPath[i]);
            }
        } else {
            std::cerr << "Error: fallback path also failed. Robot may be stuck.\n";
        }
    }

    return fullPath; 
}

*/