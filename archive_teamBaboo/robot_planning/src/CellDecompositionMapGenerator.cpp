/**
 * @file CellDecompositionMapGenerator.cpp
 * @brief Implements methods for map generation via cell decomposition from obstacles, borders, and gates.
 */

#include "../include/robotPlanning/CellDecompositionMapGenerator.hpp"
#include <algorithm>
#include <set>
#include <cmath>
#include <iostream>

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Default constructor.
 */
CellDecompositionMapGenerator::CellDecompositionMapGenerator() = default;

/**
 * @brief Set the map borders.
 * @param borders Vector of border points.
 */
void CellDecompositionMapGenerator::setBorders(const std::vector<Point>& borders) {
    borders_ = borders;
}

/**
 * @brief Set the obstacles in the map.
 * @param obstacles Vector of obstacle objects.
 */
void CellDecompositionMapGenerator::setObstacles(const std::vector<Obstacle>& obstacles) {
    obstacles_ = obstacles;
}

/**
 * @brief Set the gate positions in the map.
 * @param gates Vector of gate points.
 */
void CellDecompositionMapGenerator::setGates(const std::vector<Point>& gates) {
    gates_ = gates;
}

/**
 * @brief Generate a graph based on the current borders, obstacles, and gates using cell decomposition.
 * @param init Initial starting point.
 * @return Graph object representing the decomposed map.
 */
Graph CellDecompositionMapGenerator::generateGraph(const Point& init) {
    g_.clear();
    if (gates_.empty() || borders_.empty() || obstacles_.empty()) {
        std::cerr << "CellDecompositionMapGenerator: missing data for graph generation.\n";
        return g_;
    }
    convertCirclesToSquares();
    cellDecomposition();
    return g_;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper functions
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Comparison function for sorting points by X then Y.
 */
bool CellDecompositionMapGenerator::sortPoints(const Point &a, const Point &b) {
    return a < b;
}

/**
 * @brief Compute the centroid of a polygonal area.
 * @param area A vector of points defining the area.
 * @return Central point (centroid).
 */
Point CellDecompositionMapGenerator::getAreaCentralPoint(const std::vector<Point>& area) {
    //
    double Cx = 0, Cy = 0; 
    for (const auto &pt : area) {
        Cx += pt.getX();
        Cy += pt.getY();
    }
    return Point{Cx / area.size(), Cy / area.size()};
}

/**
 * @brief Compute the midpoint of a vertical line segment between two points.
 * @param p1 First point.
 * @param p2 Second point.
 * @return Central point on the line.
 */
Point CellDecompositionMapGenerator::getLineCentralPoint(const Point& p1, const Point& p2) {
    return Point{p1.getX(), p1.getY() - ((p1.getY() - p2.getY()) / 2)};
}

/**
 * @brief Convert all circular obstacles to square approximations.
 */
void CellDecompositionMapGenerator::convertCirclesToSquares() {
    for (auto &obs : obstacles_) {
        if (obs.getType() == CIRCLE) {
            obs.convertToSquare();
        }
    }
}

/**
 * @brief Get all unique X-values (abscissas) from obstacle boundaries.
 * @return Sorted vector of unique X-values.
 */
std::vector<double> CellDecompositionMapGenerator::getObstaclesUniqueAbscissas() {
    std::vector<double> x;
    for (auto &obs : obstacles_) {
        if (obs.getType() == CIRCLE) obs.convertToSquare();
        auto vec = obs.getAbscissas();
        x.insert(x.end(), vec.begin(), vec.end());
    }
    std::sort(x.begin(), x.end());
    x.erase(std::unique(x.begin(), x.end()), x.end());
    return x;
}

/**
 * @brief Get unique X-values from both obstacles and border points.
 * @return Sorted vector of unique X-values.
 */
std::vector<double> CellDecompositionMapGenerator::getObstaclesAndBordersUniqueAbscissas() {
    auto x = getObstaclesUniqueAbscissas();
    for (const auto &b : borders_) x.push_back(b.getX());
    std::sort(x.begin(), x.end());
    x.erase(std::unique(x.begin(), x.end()), x.end());
    return x;
}

/**
 * @brief Get all points forming the obstacles.
 * @return Vector of obstacle vertices.
 */
std::vector<Point> CellDecompositionMapGenerator::getObstaclesPoints() {
    std::vector<Point> pts;
    for (auto &obs : obstacles_) {
        if (obs.getType() == CIRCLE) obs.convertToSquare();
        auto verts = obs.getVertices();
        pts.insert(pts.end(), verts.begin(), verts.end());
    }
    return pts;
}

/**
 * @brief Get all unique and sorted points from obstacles and borders.
 * @return Sorted vector of combined points.
 */
std::vector<Point> CellDecompositionMapGenerator::getSortedObstaclesAndBordersPoints() {
    auto obsPts = getObstaclesPoints();
    std::vector<Point> unified;
    std::set_union(obsPts.begin(), obsPts.end(), borders_.begin(), borders_.end(), std::back_inserter(unified));
    std::sort(unified.begin(), unified.end());
    return unified;
}

/**
 * @brief Get all points located at a specific X-coordinate.
 * @param x The X-coordinate.
 * @return Vector of matching points.
 */
std::vector<Point> CellDecompositionMapGenerator::getPointsAtGivenAbscissa(double x) {
    std::vector<Point> result;
    auto pts = getSortedObstaclesAndBordersPoints();
    for (const auto &p : pts)
        if (p.getX() == x)
            result.push_back(p);
    return result;
}

/**
 * @brief Check if a point lies within any obstacle.
 * @param p Point to check.
 * @return True if inside an obstacle, false otherwise.
 */
bool CellDecompositionMapGenerator::isInsideObstacles(const Point &p) {
    for (auto &obs : obstacles_)
        if (obs.isInsideObstacle(p))
            return true;
    return false;
}

/**
 * @brief Check if an edge (composed of 4 corner points) intersects any obstacle.
 * @param p1 First corner.
 * @param p2 Second corner.
 * @param p3 Third corner.
 * @param p4 Fourth corner.
 * @param extr External check flag.
 * @return True if the edge belongs to an obstacle.
 */
bool CellDecompositionMapGenerator::edgeBelongsToObstacle(const Point &p1, const Point &p2,
                                                         const Point &p3, const Point &p4, bool extr) {
    for (auto &obs : obstacles_)
        if (obs.belongsToObstacle(p1, p2, p3, p4, extr))
            return true;
    return false;
}

/**
 * @brief Get intersection points between the vertical line x=constant and borders.
 * @param x X-coordinate of the line.
 * @return Vector of intersection points.
 */
std::vector<Point> CellDecompositionMapGenerator::getIntersectionWithBorders(double x) {
    std::vector<Point> result;
    std::vector<Point> loop = borders_;
    loop.push_back(borders_.front());
    for (size_t i = 0; i + 1 < loop.size(); ++i) {
        auto &p1 = loop[i], &p2 = loop[i + 1];
        if (p1.getY() == p2.getY() && (x < p1.getX()) != (x < p2.getX()))
            result.push_back({x, p1.getY()});
        if (p1.getX() != p2.getX()) {
            double y = ((p2.getY() - p1.getY()) / (p2.getX() - p1.getX())) * (x - p1.getX()) + p1.getY();
            if ((y < p1.getY()) != (y < p2.getY()) && (x < p1.getX()) != (x < p2.getX()))
                result.push_back({x, y});
        }
    }
    return result;
}

/**
 * @brief Get intersection points between vertical line x=constant and obstacles.
 * @param x X-coordinate of the line.
 * @return Vector of intersection points.
 */
std::vector<Point> CellDecompositionMapGenerator::getIntersectionWithObstacles(double x) {
    std::vector<Point> result;
    for (auto &obs : obstacles_) {
        auto pts = obs.isIntersecting(x);
        std::set_union(result.begin(), result.end(), pts.begin(), pts.end(), std::back_inserter(result));
    }
    return result;
}

/**
 * @brief Find nearest points to a reference point based on minimal Y-offset.
 * @param pts Set of candidate points.
 * @param p Reference point.
 * @return Vector of nearest points by Y.
 */
std::vector<Point> CellDecompositionMapGenerator::nearestByYOffset(const std::vector<Point>& pts,
                                                                   const Point &p) {
    std::vector<Point> best;
    if (pts.empty()) return {{0, 0}};
    double minOffset = std::abs(pts.front().getY() - p.getY());
    for (auto &q : pts)
        minOffset = std::min(minOffset, std::abs(q.getY() - p.getY()));
    for (auto &q : pts)
        if (std::abs(q.getY() - p.getY()) == minOffset)
            best.push_back(q);
    return best;
}

/**
 * @brief Compute Euclidean distance between two points.
 * @param a First point.
 * @param b Second point.
 * @return Distance as double.
 */
double CellDecompositionMapGenerator::pointDistance(const Point& a, const Point& b) const {
    double dx = a.getX() - b.getX();
    double dy = a.getY() - b.getY();
    return std::sqrt(dx * dx + dy * dy);
}

// ─────────────────────────────────────────────────────────────────────────────
// Main CellDecomposition function
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Perform cell decomposition and build a connectivity graph based on borders and obstacles.
 */
void CellDecompositionMapGenerator::cellDecomposition() {
    
    auto allXs = getObstaclesAndBordersUniqueAbscissas();
    std::vector<double> uniqueXs;
    double minXGap = 0.1;
    for (double x : allXs) {
        if (uniqueXs.empty() || std::abs(x - uniqueXs.back()) > minXGap) {
            uniqueXs.push_back(x);
        }
    }

    std::vector<Point> leftPts, rightPts;
    std::map<Point, std::pair<Point, Point>> mapLeft, mapRight;
    std::vector<Point> connectionPoints;

    auto pts = getPointsAtGivenAbscissa(uniqueXs.front());
    if (pts.size() == 1) {
        mapLeft[pts[0]] = {pts[0], pts[0]};
        leftPts.push_back(pts[0]);
    } else {
        auto center = getLineCentralPoint(pts[0], pts[1]);
        mapLeft[center] = {pts[0], pts[1]};
        leftPts.push_back(center);
    }

    for (size_t i = 1; i < uniqueXs.size(); ++i) {
        double x = uniqueXs[i];
        pts = getPointsAtGivenAbscissa(x);
        if (i < uniqueXs.size() - 1) {
            auto ib = getIntersectionWithBorders(x);
            pts.insert(pts.end(), ib.begin(), ib.end());
        }
        auto io = getIntersectionWithObstacles(x);
        pts.insert(pts.end(), io.begin(), io.end());

        // Sort and remove near-duplicates
        std::sort(pts.begin(), pts.end());
        pts.erase(std::unique(pts.begin(), pts.end(),
                    [this](const Point& a, const Point& b) {
                        return pointDistance(a, b) < 1e-3;
                    }), pts.end());

        if (i < uniqueXs.size() - 1) {
            for (size_t j = 0; j + 1 < pts.size(); ++j) {
                // Filter: skip if segment is too short
                if (pointDistance(pts[j],pts[j + 1]) < 2) continue;

                auto mid = getLineCentralPoint(pts[j], pts[j + 1]);

                // Filter: skip if mid is too close to previous
                if (!rightPts.empty() && pointDistance(mid, rightPts.back()) < 1.5) continue;

                if (isInsideObstacles(mid)) continue;
                std::vector<Point> nearestPts = nearestByYOffset(leftPts, mid);

                for (const auto& nearest : nearestPts) {
                    Point area = getAreaCentralPoint({mapLeft[nearest].first,
                                                     mapLeft[nearest].second,
                                                     pts[j], pts[j + 1]});
                    g_.addVertice(area);
                    if (i > 1) g_.addEdge(nearest, area);

                    if (mapRight.find(mid) == mapRight.end()) {
                        mapRight[mid] = {pts[j], pts[j + 1]};
                        rightPts.push_back(mid);
                        g_.addVertice(mid);
                    }
                    g_.addEdge(area, mid);

                    if (i == 1) {
                        connectionPoints.push_back(area);
                        if (j == pts.size() - 2) {
                            for (size_t z = 0; z + 1 < connectionPoints.size(); z++) {
                                g_.addEdge(connectionPoints[z], connectionPoints[z + 1]);
                            }
                        }
                    }
                }
            }
        } else {
            connectionPoints.clear();
            for (auto& lp : leftPts) {
                auto pr = mapLeft[lp];
                auto area = getAreaCentralPoint({pr.first, pr.second, pts.front(), pts.back()});
                g_.addVertice(area);
                connectionPoints.push_back(area);
                if (!isInsideObstacles(lp)) g_.addEdge(area, lp);
                for (size_t z = 0; z + 1 < connectionPoints.size(); z++) {
                    g_.addEdge(connectionPoints[z], connectionPoints[z + 1]);
                }
            }
        }
        leftPts = rightPts;
        mapLeft = mapRight;
        rightPts.clear();
        mapRight.clear();
    }
}
