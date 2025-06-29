
// SampleBasedMapGenerator.cpp
#include "../include/robotPlanning/SampleBasedMapGenerator.hpp"
#include <iostream>
#include <random>

const float minDistObs = 0.55f; // Minimum distance to obstacles
const float minDistBor = 0.6f;  // Minimum distance to obstacles

SampleBasedMapGenerator::SampleBasedMapGenerator()
{
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
}

float SampleBasedMapGenerator::getSearchRadius() const
{
    return 5.0f;
}

void SampleBasedMapGenerator::setGates(const std::vector<Point> &gates)
{
    gates_ = gates;
}

void SampleBasedMapGenerator::setBorders(const std::vector<Point> &borders)
{
    borders_ = borders;
}

void SampleBasedMapGenerator::setObstacles(const std::vector<Obstacle> &obstacles)
{
    obstacles_ = obstacles;
}

float SampleBasedMapGenerator::getRandomPosition(float middle, float r) const
{
    /* return (middle - r) + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * r))); */
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(middle - r, middle + r);
    return dis(gen);
}

Point SampleBasedMapGenerator::getRandomPoint(const Point &initP, float radius) const
{
    Point newP;

    while (true)
    {
        float x = getRandomPosition(initP.getX(), radius);
        float y = getRandomPosition(initP.getY(), radius);
        Point newP{x, y};

        // 1. Must be inside border
        if (!isInsideBorder(newP) || isTooCloseToBorder(newP, minDistBor))
            continue;

        // 2. Must not be inside or too close to any obstacle
        bool valid = true;
        for (const Obstacle &obs : obstacles_)
        {
            if (obs.isInsideObstacle(newP) || obs.isTooCloseToObstacle(newP, minDistObs))
            {
                valid = false;
                break;
            }
        }

        if (valid)
        {
            std::cout << "Generated point: (" << newP.getX() << ", " << newP.getY() << ")\n";
            return newP;
        }
    }

    return newP;
}

bool SampleBasedMapGenerator::isInsideBorder(const Point &p) const
{
    int n = static_cast<int>(borders_.size());
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++)
    {
        const Point &pi = borders_[i];
        const Point &pj = borders_[j];
        if ((pi.getY() > p.getY()) != (pj.getY() > p.getY()) &&
            (p.getX() < (pj.getX() - pi.getX()) * (p.getY() - pi.getY()) / (pj.getY() - pi.getY()) + pi.getX()))
        {
            inside = !inside;
        }
    }
    return inside;
}

bool SampleBasedMapGenerator::isTooCloseToBorder(const Point &p, double margin) const
{
    if (borders_.size() < 2)
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

    for (size_t i = 0; i < borders_.size(); ++i)
    {
        const Point &a = borders_[i];
        const Point &b = borders_[(i + 1) % borders_.size()];
        double dist = pointToSegmentDistance(p, a, b);
        if (dist < margin)
        {
            return true;
        }
    }

    return false;
}

bool SampleBasedMapGenerator::isInsideAnyObstacle(const Point &p) const
{
    for (const auto &obs : obstacles_)
    {
        if (obs.isTooCloseToObstacle(p, minDistObs) || obs.isInsideObstacle(p))
            return true;
    }
    return false;
}

bool SampleBasedMapGenerator::isReachedGate(const Point &p) const
{
    for (const auto &gate : gates_)
    {
        if (computeDistance(gate, p) <= 0.0f)
            return true;
    }
    return false;
}

float SampleBasedMapGenerator::computeDistance(const Point &a, const Point &b) const
{
    float dx = b.getX() - a.getX();
    float dy = b.getY() - a.getY();
    return dx * dx + dy * dy;
}

Point SampleBasedMapGenerator::findNearest(const Graph &graph, const Point &p) const
{
    const auto &verts = graph.getVertices();
    Point nearest = verts.front();
    float minD = computeDistance(nearest, p);
    for (const auto &v : verts)
    {
        float d = computeDistance(v, p);
        if (d < minD)
        {
            minD = d;
            nearest = v;
        }
    }
    return nearest;
}

bool SampleBasedMapGenerator::collidesWithObstacleOrBorder(const Point &p1, const Point &p2) const
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

        for (const auto &obs : obstacles_)
        {
            if (obs.isTooCloseToObstacle(interp, minDistObs) || obs.isInsideObstacle(interp))
            {
                return true;
            }
        }
    }

    return false; // No collision detected
}

void SampleBasedMapGenerator::connectVisibleEdges(Graph &graph, const Point &newP) const
{
    const auto &verts = graph.getVertices();
    for (const auto &v : verts)
    {
        if (v == newP)
            continue;

        if (!graph.edgeExists(newP, v) && !collidesWithObstacleOrBorder(newP, v))
        {
            graph.addEdge(newP, v);
        }
    }
}

// attempt to generate a graph based on random sampling
Graph SampleBasedMapGenerator::generateGraph(const Point &init)
{
    G_.clear();

    if (gates_.empty() || borders_.empty() || obstacles_.empty())
    {
        std::cerr << "SampleBasedMapGenerator: missing data for graph generation.\n";
        return G_;
    }

    const int maxIterations = 90;

    // init!
    Point initP = {init.getX(), init.getY()};
    int count = 0;

    while (count < maxIterations)
    {
        Point newP = getRandomPoint(initP, getSearchRadius());
        if (isInsideAnyObstacle(newP) || isTooCloseToBorder(newP, minDistBor))
            continue;

        // Check for duplicates
        if (G_.containsVertex(newP))
            continue;

        ++count;

        if (G_.getVertices().empty())
        {
            G_.addEdge(initP, newP);
        }
        else
        {
            connectVisibleEdges(G_, newP);

            /*
            Point nearest = findNearest(G_, newP);
             if (!G_.edgeExists(nearest, newP)) {
                G_.addEdge(nearest, newP);
            } */
        }

        initP = newP; // Update initP to the last added point
        
    }

    return G_;
}
