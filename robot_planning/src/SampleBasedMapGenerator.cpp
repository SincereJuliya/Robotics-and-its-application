
// SampleBasedMapGenerator.cpp
#include "../include/robotPlanning/SampleBasedMapGenerator.hpp"
#include <iostream>

SampleBasedMapGenerator::SampleBasedMapGenerator() {
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
}

float SampleBasedMapGenerator::getSearchRadius() const {
    return 10.0f;   
}

float SampleBasedMapGenerator::getRandomPosition(float middle, float r) const {
    return (middle - r) + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * r)));
}

Point SampleBasedMapGenerator::getRandomPoint(const Point& initP, float radius) const {
    Point newP;
    do {
        float x = getRandomPosition(initP.getX(), radius);
        float y = getRandomPosition(initP.getY(), radius);
        newP = Point{x, y};
    } while (!isInsideBorder(newP) || isInsideAnyObstacle(newP));
    return newP;
}

bool SampleBasedMapGenerator::isInsideBorder(const Point& p) const {
    int n = static_cast<int>(borders_.size());
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Point& pi = borders_[i];
        const Point& pj = borders_[j];
        if ((pi.getY() > p.getY()) != (pj.getY() > p.getY()) &&
            (p.getX() < (pj.getX() - pi.getX()) * (p.getY() - pi.getY()) / (pj.getY() - pi.getY()) + pi.getX())) {
            inside = !inside;
        }
    }
    return inside;
}

bool SampleBasedMapGenerator::isInsideAnyObstacle(const Point& p) const {
    for (const auto& obs : obstacles_) {
        if (obs.isInsideObstacle(p)) return true;
    }
    return false;
}

bool SampleBasedMapGenerator::isReachedGate(const Point& p) const {
    for (const auto& gate : gates_) {
        if (computeDistance(gate, p) < 1.0f) return true;
    }
    return false;
}

float SampleBasedMapGenerator::computeDistance(const Point& a, const Point& b) const {
    float dx = b.getX() - a.getX();
    float dy = b.getY() - a.getY();
    return dx*dx + dy*dy;
}

Point SampleBasedMapGenerator::findNearest(const Graph& graph, const Point& p) const {
    const auto& verts = graph.getVertices();
    Point nearest = verts.front();
    float minD = computeDistance(nearest, p);
    for (const auto& v : verts) {
        float d = computeDistance(v, p);
        if (d < minD) {
            minD = d;
            nearest = v;
        }
    }
    return nearest;
}

void SampleBasedMapGenerator::setGates(const std::vector<Point>& gates) {
    gates_ = gates;
}

void SampleBasedMapGenerator::setBorders(const std::vector<Point>& borders) {
    borders_ = borders;
}

void SampleBasedMapGenerator::setObstacles(const std::vector<Obstacle>& obstacles) {
    obstacles_ = obstacles;
}

Graph SampleBasedMapGenerator::generateGraph() {
    G_.clear();
    if (gates_.empty() || borders_.empty() || obstacles_.empty()) {
        std::cerr << "SampleBasedMapGenerator: missing data for graph generation.\n";
        return G_;
    }

    const int maxIterations = 4;
    // init!
    Point initP{0.0f, 0.0f};
    int count = 0;

    while (count < maxIterations) {
        Point newP = getRandomPoint(initP, getSearchRadius());
        if (isInsideAnyObstacle(newP)) continue;
        ++count;

        if (G_.getVertices().empty()) {
            G_.addEdge(initP, newP);
        } else {
            Point nearest = findNearest(G_, newP);
            G_.addEdge(nearest, newP);
        }

        if (isReachedGate(newP)) break;
    }

    return G_;
}
